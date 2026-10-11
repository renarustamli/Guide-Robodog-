import AVFoundation
import Combine
import Foundation
import Speech

/// Voice commands from the handle microphone.
/// Encoder click -> board sends LISTEN -> we send RECORD -> board records 3 s and streams it here ->
/// Apple speech recognition (on the phone) -> a drive word goes back as VOICE:<epoch>:<word>,
/// "guide me to X" starts a walking route.
final class VoiceCommand: ObservableObject {
    @Published var status = "Click the encoder on the handle, then speak."
    @Published var heard = ""
    @Published var reply = ""
    /// Speak replies through the phone speaker. Off = buzzer feedback only.
    @Published var spokenReplies = true
    /// Size and loudness of the last clip, to check the handle microphone.
    @Published var clipInfo = ""
    @Published var hasClip = false

    /// Sends a command to the board. Set by the view.
    var send: ((String) -> Void)?
    /// Starts a walking route to a place name. Set by the view.
    var startRoute: ((String) -> Void)?
    /// Starts a scripted drive route if the place is one of the dummy destinations. Returns false if not.
    var startDummyRoute: ((String) -> Bool)?
    /// A found Apple Maps route is waiting for the user to say yes or no.
    var routePending: (() -> Bool)?
    var confirmRoute: (() -> Void)?
    /// Drops a found route that is waiting for yes or no.
    var cancelPendingRoute: (() -> Void)?
    /// Abandons a scripted drive route. Called on a spoken stop.
    var cancelRoute: (() -> Void)?

    private let recognizer = SFSpeechRecognizer(locale: Locale(identifier: "en-GB"))
    private let synthesizer = AVSpeechSynthesizer()
    private var task: SFSpeechRecognitionTask?
    private var audio = Data()
    private var collecting = false
    /// RECORD sent, AUDIO_BEGIN not yet received.
    private var awaitingClip = false
    /// Speech recognition running on the last clip.
    private var recognizing = false
    /// When the current click started. A clip takes about 8 s end to end, so after 15 s
    /// we assume it was lost (board restarted, Bluetooth dropped) and accept clicks again.
    private var busySince = Date.distantPast
    /// Drive word sent to the board, waiting for its STATE line to confirm, with the reply to speak then.
    private var pendingDrive: (direction: String, reply: String)?
    private var driveTimeout: DispatchWorkItem?
    private var epoch: UInt32 = 0
    private var lastClip = Data()
    private var player: AVAudioPlayer?

    init() {
        SFSpeechRecognizer.requestAuthorization { _ in }
    }

    /// Asks the board for a clip. Same as clicking the encoder.
    func listen() {
        // One click at a time: ignore extra clicks while a clip is on its way or being recognised.
        let busy = awaitingClip || collecting || recognizing
        if busy && Date().timeIntervalSince(busySince) < 15 { return }
        reset()
        busySince = Date()
        awaitingClip = true
        send?("RECORD")
    }

    func handle(line: String) {
        if line.hasPrefix("STATE:"), let pending = pendingDrive {
            // STATE:<epoch>,<speed>,<direction>,<stop held>
            let fields = line.dropFirst(6).split(separator: ",")
            if fields.count == 4, fields[2] == pending.direction {
                finishDrive()
                say(pending.reply)
            }
        } else if line.hasPrefix("ERROR:Stale"), pendingDrive != nil {
            finishDrive()
            say("The robot was stopped while I was listening. Please say it again.")
        } else if line.hasPrefix("LISTEN:") {
            listen()
        } else if line.hasPrefix("AUDIO_BEGIN:") {
            epoch = UInt32(line.dropFirst(12)) ?? 0
            awaitingClip = false
            audio = Data()
            collecting = true
            status = "Listening… speak now (3 seconds)."
        } else if line.hasPrefix("AUDIO_END:"), collecting {
            collecting = false
            // AUDIO_END:<epoch>:<bytes>
            let parts = line.split(separator: ":")
            let expected = parts.count == 3 ? Int(parts[2]) ?? 0 : 0
            keepClip(audio, expected: expected)
            // Bluetooth can drop a few packets. A few missing milliseconds don't matter for speech,
            // so recognise anything at least 95% complete.
            guard expected > 0, audio.count * 100 >= expected * 95 else {
                status = "Too much audio was lost over Bluetooth. Click the encoder to try again."
                return
            }
            if audio.count % 2 == 1 { audio.removeLast() }
            recognize(audio)
        } else if line.hasPrefix("ERROR:"), awaitingClip || collecting {
            // "Microphone busy" just means an extra click; the clip in progress is still fine.
            if line.contains("busy") && collecting { return }
            awaitingClip = false
            collecting = false
            status = String(line.dropFirst(6))
        } else if line == "AVOID:LEFT" || line == "AVOID:RIGHT" {
            say("Obstacle ahead. Going around to the \(line == "AVOID:LEFT" ? "left" : "right").")
        } else if line == "AVOID:Rejoining" {
            say("Rejoining the path.")
        } else if line == "AVOID:Done" {
            say("Back on the path.")
        } else if line.hasPrefix("REFUSED:") {
            // Intelligent disobedience: the board would not move forward.
            finishDrive()
            say(line.contains("Camera is off") ? "I can't go forward. Start the camera first." : "I can't go forward.")
        }
    }

    /// Plays the last clip through the phone speaker, to hear what the handle microphone captured.
    func playLastClip() {
        guard !lastClip.isEmpty else { return }
        try? AVAudioSession.sharedInstance().setCategory(.playback)
        player = try? AVAudioPlayer(data: Self.wav(from: lastClip))
        player?.play()
    }

    private func keepClip(_ pcm: Data, expected: Int) {
        lastClip = pcm
        hasClip = !pcm.isEmpty
        var peak: Int32 = 0
        pcm.withUnsafeBytes { raw in
            for sample in raw.bindMemory(to: Int16.self) { peak = max(peak, abs(Int32(Int16(littleEndian: sample)))) }
        }
        let seconds = Double(pcm.count) / 32000
        clipInfo = String(format: "Last clip: %.1f s, %d of %d bytes, peak level %.0f%%",
                          seconds, pcm.count, expected, Double(peak) / 327.68)
    }

    private func finishDrive() {
        driveTimeout?.cancel()
        driveTimeout = nil
        pendingDrive = nil
    }

    /// Forget any clip in progress, e.g. after Bluetooth reconnects.
    func reset() {
        awaitingClip = false
        collecting = false
        recognizing = false
        task?.cancel()
        task = nil
        audio = Data()
    }

    func append(audio chunk: Data) {
        if collecting { audio.append(chunk) }
    }

    func say(_ text: String) {
        reply = text
        guard spokenReplies else { return }
        try? AVAudioSession.sharedInstance().setCategory(.playback, options: .duckOthers)
        synthesizer.stopSpeaking(at: .immediate)
        synthesizer.speak(AVSpeechUtterance(string: text))
    }

    private func recognize(_ pcm: Data) {
        guard let recognizer, recognizer.isAvailable else {
            status = "Speech recognition unavailable. Allow it in Settings → Robodog."
            return
        }
        guard let buffer = Self.floatBuffer(from: pcm) else { return }
        status = "Recognising…"
        recognizing = true
        let request = SFSpeechAudioBufferRecognitionRequest()
        request.shouldReportPartialResults = false
        if recognizer.supportsOnDeviceRecognition { request.requiresOnDeviceRecognition = true }
        request.append(buffer)
        request.endAudio()
        let clipEpoch = epoch
        task = recognizer.recognitionTask(with: request) { [weak self] result, error in
            DispatchQueue.main.async {
                guard let self else { return }
                if result?.isFinal == true || error != nil { self.recognizing = false }
                if let result, result.isFinal {
                    self.interpret(result.bestTranscription.formattedString, epoch: clipEpoch)
                } else if let error {
                    self.heard = ""
                    self.status = "Didn't catch that (\(error.localizedDescription)). Click the encoder to try again."
                }
            }
        }
    }

    private func interpret(_ text: String, epoch: UInt32) {
        heard = text
        status = "Click the encoder on the handle, then speak."
        let words = text.lowercased()

        // Stop wins over everything, whatever else was said.
        if Self.has(words, "stop|halt|wait") {
            cancelRoute?()
            cancelPendingRoute?()
            send?("VOICE:\(epoch):stop")
            say("Stopping.")
            return
        }
        // A found route is waiting for yes or no; anything else falls through to the normal commands.
        if routePending?() == true {
            if Self.has(words, "yes|yeah|yep|okay|ok|sure|confirm|go ahead") {
                confirmRoute?()
                return
            }
            if Self.has(words, "no|nope|cancel|never mind") {
                cancelPendingRoute?()
                return
            }
        }
        if let place = Self.destination(in: words) {
            if startDummyRoute?(place) == true { return }
            startRoute?(place)
            say("Finding a route to \(place).")
            return
        }
        let drive: [(pattern: String, word: String, reply: String)] = [
            ("left", "left", "Turning left."),
            ("right", "right", "Turning right."),
            ("back|backwards|reverse", "reverse", "Reversing."),
            ("forward|forwards|go|walk|straight|ahead", "forward", "Going forward."),
        ]
        if let match = drive.first(where: { Self.has(words, $0.pattern) }) {
            // Reply only once the board confirms the move (or refuses it), not just because we heard the word.
            finishDrive()
            pendingDrive = (match.word, match.reply)
            let timeout = DispatchWorkItem { [weak self] in
                guard let self, self.pendingDrive != nil else { return }
                self.pendingDrive = nil
                self.say("The robot didn't respond.")
            }
            driveTimeout = timeout
            DispatchQueue.main.asyncAfter(deadline: .now() + 2, execute: timeout)
            send?("VOICE:\(epoch):\(match.word)")
            return
        }
        say("Sorry, I didn't understand. Say forward, left, right, stop, or guide me to a place.")
    }

    private static func has(_ text: String, _ pattern: String) -> Bool {
        text.range(of: "\\b(\(pattern))\\b", options: .regularExpression) != nil
    }

    /// "guide me to Tesco", "take me to the station", "navigate to Boots" -> place name.
    private static func destination(in text: String) -> String? {
        let pattern = "\\b(?:guide me to|take me to|navigate to|directions to|walk to|go to)\\s+(?:the\\s+)?(.+)"
        guard let regex = try? NSRegularExpression(pattern: pattern),
              let match = regex.firstMatch(in: text, range: NSRange(text.startIndex..., in: text)),
              let range = Range(match.range(at: 1), in: text) else { return nil }
        let place = text[range].trimmingCharacters(in: .punctuationCharacters.union(.whitespaces))
        return place.isEmpty ? nil : place
    }

    /// 16-bit mono PCM at 16 kHz -> WAV file bytes.
    private static func wav(from pcm: Data) -> Data {
        var header = Data()
        func append<T: FixedWidthInteger>(_ value: T) { withUnsafeBytes(of: value.littleEndian) { header.append(contentsOf: $0) } }
        header.append(contentsOf: Array("RIFF".utf8)); append(UInt32(36 + pcm.count))
        header.append(contentsOf: Array("WAVEfmt ".utf8)); append(UInt32(16)); append(UInt16(1)); append(UInt16(1))
        append(UInt32(16000)); append(UInt32(32000)); append(UInt16(2)); append(UInt16(16))
        header.append(contentsOf: Array("data".utf8)); append(UInt32(pcm.count))
        return header + pcm
    }

    /// 16-bit little-endian mono PCM at 16 kHz -> Float32 buffer for the recogniser.
    private static func floatBuffer(from pcm: Data) -> AVAudioPCMBuffer? {
        let frames = pcm.count / 2
        guard frames > 0,
              let format = AVAudioFormat(commonFormat: .pcmFormatFloat32, sampleRate: 16000, channels: 1, interleaved: false),
              let buffer = AVAudioPCMBuffer(pcmFormat: format, frameCapacity: AVAudioFrameCount(frames)),
              let output = buffer.floatChannelData?[0] else { return nil }
        pcm.withUnsafeBytes { raw in
            let samples = raw.bindMemory(to: Int16.self)
            for i in 0..<frames { output[i] = Float(Int16(littleEndian: samples[i])) / 32768 }
        }
        buffer.frameLength = AVAudioFrameCount(frames)
        return buffer
    }
}
