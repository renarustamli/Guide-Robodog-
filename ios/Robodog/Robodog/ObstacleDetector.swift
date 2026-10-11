import AVFoundation
import Combine
import CoreML
import SwiftUI
import Vision

/// Rear-camera object detection. Same rule as browser/Camera Test.html: a recognized
/// object overlapping the centre zone means BLOCK.
final class ObstacleDetector: NSObject, ObservableObject {
    /// Name of the Core ML model file you drag into Xcode (without .mlmodel).
    static let modelName = "YOLOv3Tiny"

    @Published var isRunning = false
    @Published var blocked = false
    @Published var objects: [String] = []
    @Published var error: String?

    let session = AVCaptureSession()
    /// Called on the main thread after every analysed frame (about 5 per second). true = blocked.
    var onFrame: ((Bool) -> Void)?

    private let queue = DispatchQueue(label: "robodog.camera")
    private var request: VNCoreMLRequest?
    private var configured = false
    private var lastFrame = Date.distantPast
    private var lastGood = Date.distantPast
    private var held: [VNRecognizedObjectObservation] = []

    // Middle 40% wide, 70% tall (Vision uses normalized coordinates).
    private let zone = CGRect(x: 0.3, y: 0.15, width: 0.4, height: 0.7)

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
                self.blocked = false
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

extension ObstacleDetector: AVCaptureVideoDataOutputSampleBufferDelegate {
    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        guard Date().timeIntervalSince(lastFrame) >= 0.2, let request,
              let pixels = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        lastFrame = Date()

        // .right = phone held upright (portrait).
        try? VNImageRequestHandler(cvPixelBuffer: pixels, orientation: .right).perform([request])
        let good = (request.results as? [VNRecognizedObjectObservation] ?? []).filter { $0.confidence >= 0.6 }
        // Hold detections briefly so one missed frame doesn't flicker to CLEAR.
        if !good.isEmpty {
            held = good
            lastGood = Date()
        } else if Date().timeIntervalSince(lastGood) > 0.7 {
            held = []
        }

        let isBlocked = held.contains { $0.boundingBox.intersects(zone) }
        let labels = held.map { "\($0.labels.first?.identifier ?? "object") · \(Int($0.confidence * 100))%" }
        DispatchQueue.main.async {
            guard self.isRunning else { return }
            self.blocked = isBlocked
            self.objects = labels
            self.onFrame?(isBlocked)
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
