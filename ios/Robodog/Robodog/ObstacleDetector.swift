import AVFoundation
import Combine
import CoreML
import SwiftUI
import Vision

/// Rear-camera object detection. Same rule as browser/Camera Test.html: a recognized
/// object overlapping the centre zone means the path is blocked. If one side of the view is free,
/// the board is told to go around that way (CAM:AVOID:LEFT / CAM:AVOID:RIGHT) instead of stopping.
final class ObstacleDetector: NSObject, ObservableObject {
    enum Verdict: Equatable {
        case clear, avoidLeft, avoidRight, blocked

        /// The command sent to the board for this frame.
        var command: String {
            switch self {
            case .clear: return "CAM:CLEAR"
            case .avoidLeft: return "CAM:AVOID:LEFT"
            case .avoidRight: return "CAM:AVOID:RIGHT"
            case .blocked: return "CAM:BLOCK"
            }
        }
    }

    /// Name of the Core ML model file you drag into Xcode (without .mlmodel).
    static let modelName = "YOLOv3Tiny"

    @Published var isRunning = false
    @Published var verdict = Verdict.clear
    /// An object counts as an obstacle when it fills at least this fraction of the picture's height.
    /// The camera can't measure distance, so closeness is judged by how big the object looks:
    /// higher = only very close objects, lower = further objects too. Set from the slider on screen.
    @Published var nearHeight: Double = ObstacleDetector.defaultNearHeight {
        didSet {
            let height = CGFloat(nearHeight)
            queue.async { self.queueNearHeight = height }
        }
    }
    static let defaultNearHeight = 0.33
    /// Detections less certain than this are ignored.
    static let minConfidence: Float = 0.5
    var blocked: Bool { verdict != .clear }
    @Published var objects: [String] = []
    @Published var error: String?

    let session = AVCaptureSession()
    /// Called on the main thread after every analysed frame (about 5 per second).
    var onFrame: ((Verdict) -> Void)?

    private let queue = DispatchQueue(label: "robodog.camera")
    private var request: VNCoreMLRequest?
    private var configured = false
    private var lastFrame = Date.distantPast
    private var lastGood = Date.distantPast
    private var held: [VNRecognizedObjectObservation] = []
    /// Copy of nearHeight, read on the camera queue only.
    private var queueNearHeight = CGFloat(ObstacleDetector.defaultNearHeight)

    // Middle 40% wide, 70% tall (Vision uses normalized coordinates).
    private let zone = CGRect(x: 0.3, y: 0.15, width: 0.4, height: 0.7)
    // The strips either side of the centre, same height. Free = nothing recognised in it.
    private let leftZone = CGRect(x: 0, y: 0.15, width: 0.3, height: 0.7)
    private let rightZone = CGRect(x: 0.7, y: 0.15, width: 0.3, height: 0.7)

    func start() {
        AVCaptureDevice.requestAccess(for: .video) { granted in
            guard granted else {
                DispatchQueue.main.async { self.error = "Camera permission denied. Allow it in Settings." }
                return
            }
            self.queue.async {
                do {
                    try self.configureIfNeeded()
                } catch {
                    DispatchQueue.main.async { self.error = error.localizedDescription }
                    return
                }
                self.session.startRunning()
                DispatchQueue.main.async {
                    self.isRunning = true
                    self.error = nil
                }
            }
        }
    }

    func stop() {
        queue.async {
            self.session.stopRunning()
            self.held = []
            DispatchQueue.main.async {
                self.isRunning = false
                self.verdict = .clear
                self.objects = []
            }
        }
    }

    private func configureIfNeeded() throws {
        guard !configured else { return }
        guard let url = Bundle.main.url(forResource: Self.modelName, withExtension: "mlmodelc") else {
            throw DetectorError("Model \(Self.modelName) not found. Drag \(Self.modelName).mlmodel into Xcode.")
        }
        let request = VNCoreMLRequest(model: try VNCoreMLModel(for: MLModel(contentsOf: url)))
        request.imageCropAndScaleOption = .scaleFill
        self.request = request

        guard let camera = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) else {
            throw DetectorError("No rear camera found.")
        }
        session.beginConfiguration()
        session.sessionPreset = .vga640x480
        let input = try AVCaptureDeviceInput(device: camera)
        if session.canAddInput(input) { session.addInput(input) }
        let output = AVCaptureVideoDataOutput()
        output.alwaysDiscardsLateVideoFrames = true
        output.setSampleBufferDelegate(self, queue: queue)
        if session.canAddOutput(output) { session.addOutput(output) }
        session.commitConfiguration()
        configured = true
    }
}

extension ObstacleDetector {
    /// Clear if nothing is in the centre. Otherwise go around on a free side: if both are free, the
    /// side away from the obstacles; if neither is, blocked.
    func decide(_ boxes: [CGRect]) -> Verdict {
        let ahead = boxes.filter { $0.intersects(zone) }
        guard !ahead.isEmpty else { return .clear }
        let leftFree = !boxes.contains { $0.intersects(leftZone) }
        let rightFree = !boxes.contains { $0.intersects(rightZone) }
        switch (leftFree, rightFree) {
        case (true, false): return .avoidLeft
        case (false, true): return .avoidRight
        case (false, false): return .blocked
        case (true, true):
            let centre = ahead.map(\.midX).reduce(0, +) / Double(ahead.count)
            return centre > 0.5 ? .avoidLeft : .avoidRight
        }
    }
}

extension ObstacleDetector: AVCaptureVideoDataOutputSampleBufferDelegate {
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        guard Date().timeIntervalSince(lastFrame) >= 0.2, let request,
              let pixels = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        lastFrame = Date()

        // .right = phone held upright (portrait).
        try? VNImageRequestHandler(cvPixelBuffer: pixels, orientation: .right).perform([request])
        let good = (request.results as? [VNRecognizedObjectObservation] ?? []).filter { $0.confidence >= Self.minConfidence }
        // Hold detections briefly so one missed frame doesn't flicker to CLEAR.
        if !good.isEmpty {
            held = good
            lastGood = Date()
        } else if Date().timeIntervalSince(lastGood) > 0.7 {
            held = []
        }

        // Far-away (small-looking) objects are listed but don't count as obstacles.
        let near = held.filter { $0.boundingBox.height >= queueNearHeight }
        let verdict = decide(near.map(\.boundingBox))
        let labels = held.map { observation in
            let distance = observation.boundingBox.height >= queueNearHeight ? "near" : "far"
            return "\(observation.labels.first?.identifier ?? "object") · \(Int(observation.confidence * 100))% · \(distance)"
        }
        DispatchQueue.main.async {
            guard self.isRunning else { return }
            self.verdict = verdict
            self.objects = labels
            self.onFrame?(verdict)
        }
    }
}

struct DetectorError: LocalizedError {
    let errorDescription: String?
    init(_ message: String) { errorDescription = message }
}

/// Live camera view for the screen.
struct CameraPreview: UIViewRepresentable {
    let session: AVCaptureSession

    func makeUIView(context: Context) -> PreviewView {
        let view = PreviewView()
        view.previewLayer.session = session
        view.previewLayer.videoGravity = .resizeAspectFill
        return view
    }

    func updateUIView(_ uiView: PreviewView, context: Context) {}

    final class PreviewView: UIView {
        override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
        var previewLayer: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
    }
}
