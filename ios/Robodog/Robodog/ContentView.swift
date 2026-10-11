import SwiftUI

struct ContentView: View {
    @StateObject private var ble = BLEManager()
    @StateObject private var detector = ObstacleDetector()
    @StateObject private var guide = RouteGuide()
    @State private var destination = ""

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                Text("Robodog").font(.largeTitle.bold())

                Label(ble.state, systemImage: ble.isConnected ? "dot.radiowaves.left.and.right" : "antenna.radiowaves.left.and.right.slash")
                    .font(.headline)
                Text(ble.boardStatus)
                    .font(.footnote)
                    .foregroundStyle(.secondary)
                    .accessibilityLabel("Board: \(ble.boardStatus)")

                GroupBox("Obstacle camera") {
                    VStack(alignment: .leading, spacing: 10) {
                        CameraPreview(session: detector.session)
                            .frame(height: 260)
                            .clipShape(RoundedRectangle(cornerRadius: 12))
                            .accessibilityHidden(true)
                        Text(cameraStatus).font(.title3.bold())
                        ForEach(detector.objects, id: \.self) { Text($0) }
                        if let error = detector.error {
                            Text(error).foregroundStyle(.red)
                        }
                        Button(detector.isRunning ? "Stop camera" : "Start camera", action: toggleCamera)
                            .buttonStyle(.borderedProminent)
                            .controlSize(.large)
                    }
                }

                GroupBox("Navigation") {
                    VStack(alignment: .leading, spacing: 10) {
                        TextField("Where to? e.g. Tesco", text: $destination)
                            .textFieldStyle(.roundedBorder)
                            .submitLabel(.go)
                            .onSubmit(startRoute)
                        HStack {
                            Button("Start route", action: startRoute)
                                .buttonStyle(.borderedProminent)
                                .disabled(destination.trimmingCharacters(in: .whitespaces).isEmpty)
                            Button("Demo route") { guide.playDemo() }
                                .buttonStyle(.bordered)
                            if guide.isGuiding {
                                Button("Stop", role: .destructive) { guide.stop() }
                                    .buttonStyle(.bordered)
                            }
                        }
                        .controlSize(.large)
                        Text(guide.instruction).font(.title3)
                    }
                }

                Text("Recognizes common objects only, such as people, chairs and bottles. It does not reliably find glass, walls, holes or steps. No recognized object does not mean the path is clear.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
            }
            .padding()
        }
        .onAppear {
            detector.onFrame = { blocked in ble.send(blocked ? "CAM:BLOCK" : "CAM:CLEAR") }
            guide.onCue = { cue in ble.send("NAV:" + cue) }
        }
    }

    private var cameraStatus: String {
        if !detector.isRunning { return "Camera off" }
        return detector.blocked ? "Obstacle ahead: STOP sent" : "No obstacle recognized in the centre"
    }

    private func toggleCamera() {
        if detector.isRunning {
            detector.stop()
            ble.send("CAM:OFF")
            UIApplication.shared.isIdleTimerDisabled = false
        } else {
            detector.start()
            // Screen lock would pause the camera, which stops the robot.
            UIApplication.shared.isIdleTimerDisabled = true
        }
    }

    private func startRoute() {
        let query = destination.trimmingCharacters(in: .whitespaces)
        guard !query.isEmpty else { return }
        guide.start(to: query)
    }
}
