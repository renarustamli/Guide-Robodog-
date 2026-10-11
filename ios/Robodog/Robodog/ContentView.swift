import SwiftUI

struct ContentView: View {
    @StateObject private var ble = BLEManager()
    @StateObject private var detector = ObstacleDetector()
    @StateObject private var guide = RouteGuide()
    @StateObject private var voice = VoiceCommand()
    @StateObject private var driver = RouteExecutor()
    @State private var destination = ""

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                Text("Robodog").font(.largeTitle.bold())

                Label(ble.state, systemImage: ble.isConnected ? "dot.radiowaves.left.and.right" : "antenna.radiowaves.left.and.right.slash")
                    .font(.headline)
                if let handheld = ble.handheld {
                    Text(handheldSummary(handheld)).font(.title3.bold())
                }
                Text(ble.boardStatus)
                    .font(.footnote)
                    .foregroundStyle(.secondary)
                    .accessibilityLabel("Board: \(ble.boardStatus)")

                GroupBox("Voice") {
                    VStack(alignment: .leading, spacing: 10) {
                        Text(voice.status)
                        if !voice.heard.isEmpty { Text("You: \(voice.heard)").font(.headline) }
                        if !voice.reply.isEmpty { Text("Robodog: \(voice.reply)") }
                        Button("Talk", action: voice.listen)
                            .buttonStyle(.borderedProminent)
                            .controlSize(.large)
                            .disabled(!ble.isConnected)
                        Toggle("Spoken replies", isOn: $voice.spokenReplies)
                        if voice.hasClip {
                            Text(voice.clipInfo).font(.footnote).foregroundStyle(.secondary)
                            Button("Play last recording", action: voice.playLastClip)
                                .buttonStyle(.bordered)
                        }
                    }
                }

                GroupBox("Demo destinations") {
                    VStack(alignment: .leading, spacing: 10) {
                        Text("Say \"I want to go to the supermarket\" or \"…the museum\", or tap a button.")
                            .font(.footnote)
                            .foregroundStyle(.secondary)
                        HStack {
                            ForEach(RouteExecutor.dummyRoutes.indices, id: \.self) { index in
                                let route = RouteExecutor.dummyRoutes[index].route
                                Button(route.name.replacingOccurrences(of: "the ", with: "").capitalized) {
                                    driver.start(route)
                                }
                                .buttonStyle(.bordered)
                                .disabled(!ble.isConnected || driver.isRunning)
                            }
                        }
                        .controlSize(.large)
                        Text(driver.status).font(.title3)
                        if driver.isRunning {
                            Button("Stop route", role: .destructive) { driver.cancel() }
                                .buttonStyle(.borderedProminent)
                                .controlSize(.large)
                        }
                    }
                }

                GroupBox("Obstacle camera") {
                    VStack(alignment: .leading, spacing: 10) {
                        CameraPreview(session: detector.session)
                            .frame(height: 260)
                            .clipShape(RoundedRectangle(cornerRadius: 12))
                            .accessibilityHidden(true)
                        Text(cameraStatus).font(.title3.bold())
                        VStack(alignment: .leading, spacing: 2) {
                            Text("Stop distance")
                            HStack {
                                Text("Near").font(.footnote)
                                // Right = stop further away = a smaller object height already counts.
                                Slider(value: Binding(get: { 0.75 - detector.nearHeight },
                                                      set: { detector.nearHeight = 0.75 - $0 }),
                                       in: 0.15...0.6)
                                Text("Far").font(.footnote)
                            }
                        }
                        .accessibilityElement(children: .combine)
                        .accessibilityLabel("Stop distance")
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
                            Menu("Demo route") {
                                ForEach(RouteGuide.demoRoutes.indices, id: \.self) { i in
                                    Button(RouteGuide.demoRoutes[i].name) { guide.playDemo(i) }
                                }
                            }
                                .buttonStyle(.bordered)
                            if guide.isGuiding {
                                Button("Stop", role: .destructive) { guide.stop() }
                                    .buttonStyle(.bordered)
                            }
                        }
                        .controlSize(.large)
                        Text(guide.instruction).font(.title3)
                        if guide.pendingName != nil {
                            HStack {
                                Button("Yes, go") { guide.confirmPending() }
                                    .buttonStyle(.borderedProminent)
                                Button("No", role: .cancel) { guide.cancelPending() }
                                    .buttonStyle(.bordered)
                            }
                            .controlSize(.large)
                        }
                    }
                }

                Text("Recognizes common objects only, such as people, chairs and bottles. It does not reliably find glass, walls, holes or steps. No recognized object does not mean the path is clear.")
                    .font(.footnote)
                    .foregroundStyle(.secondary)
            }
            .padding()
        }
        .onAppear {
            detector.onFrame = { verdict in ble.send(verdict.command) }
            guide.onCue = { cue in ble.send("NAV:" + cue) }
            guide.onAnnounce = { text in voice.say(text) }
            ble.onLine = { line in
                voice.handle(line: line)
                driver.handle(line: line)
            }
            ble.onAudio = { chunk in voice.append(audio: chunk) }
            ble.onConnectionChange = { connected in
                voice.reset()
                if !connected { driver.cancel(announce: "Connection lost. Route cancelled.", sendStop: false) }
            }
            driver.send = { command in ble.send(command) }
            driver.say = { text in voice.say(text) }
            voice.send = { command in ble.send(command) }
            voice.cancelRoute = { driver.cancel(announce: nil) }
            voice.startDummyRoute = { place in
                guard let route = RouteExecutor.route(for: place) else { return false }
                destination = place
                driver.start(route)
                return true
            }
            voice.routePending = { guide.pendingName != nil }
            voice.confirmRoute = { guide.confirmPending() }
            voice.cancelPendingRoute = { guide.cancelPending() }
            voice.startRoute = { place in
                destination = place
                guide.start(to: place)
            }
        }
    }

    private func handheldSummary(_ state: BLEManager.HandheldState) -> String {
        let motion = state.direction == "stop" ? "Stopped" : "Moving \(state.direction)"
        return "\(motion) · speed \(state.speed)" + (state.stopHeld ? " · stop button held" : "")
    }

    private var cameraStatus: String {
        if !detector.isRunning { return "Camera off" }
        switch detector.verdict {
        case .clear: return "No obstacle recognized in the centre"
        case .avoidLeft: return "Obstacle ahead: left side is free"
        case .avoidRight: return "Obstacle ahead: right side is free"
        case .blocked: return "Obstacle ahead, no free side: STOP"
        }
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
