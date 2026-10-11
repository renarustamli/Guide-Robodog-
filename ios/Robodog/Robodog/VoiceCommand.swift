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

    /// Sends a command to the board. Set by the view.
    var send: ((String) -> Void)?
    /// Starts a walking route to a place name. Set by the view.
    var startRoute: ((String) -> Void)?

    private let recognizer = SFSpeechRecognizer(locale: Locale(identifier: "en-GB"))
    private let synthesizer = AVSpeechSynthesizer()
    private var task: SFSpeechRecognitionTask?
    private var audio = Data()
    private var collecting = false
    /// RECORD sent, AUDIO_BEGIN not yet received.
    private var awaitingClip = false
    private var epoch: UInt32 = 0

    init() {
        SFSpeechRecognizer.requestAuthorization { _ in }
    }

    /// Asks the board for a clip. Same as clicking the encoder.
    func listen() {
        awaitingClip = true
        send?("RECORD")
    }

    func handle(line: String) {
        if line.hasPrefix("LISTEN:") {
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
            guard parts.count == 3, Int(parts[2]) == audio.count else {
                status = "Some audio was lost over Bluetooth. Click the encoder to try again."
                return
            }
            recognize(audio)
        } else if line.hasPrefix("ERROR:"), awaitingClip || collecting {
            awaitingClip = false
            collecting = false
            status = String(line.dropFirst(6))
        } else if line.hasPrefix("REFUSED:") {
            // Intelligent disobedience: the board would not move forward.
            say("I can't go forward. The path ahead isn't clear.")
        }
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
        let request = SFSpeechAudioBufferRecognitionRequest()
        request.shouldReportPartialResults = false
        if recognizer.supportsOnDeviceRecognition { request.requiresOnDeviceRecognition = true }
        request.append(buffer)
        request.endAudio()
        let clipEpoch = epoch
        task = recognizer.recognitionTask(with: request) { [weak self] result, error in
            DispatchQueue.main.async {
                guard let self else { return }
                if let result, result.isFinal {
                    self.interpret(result.bestTranscription.formattedString, epoch: clipEpoch)
                } else if error != nil {
                    self.heard = ""
                    self.status = "Didn't catch that. Click the encoder to try again."
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
            send?("VOICE:\(epoch):stop")
            say("Stopping.")
            return
        }
        if let place = Self.destination(in: words) {
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
            send?("VOICE:\(epoch):\(match.word)")
            say(match.reply)
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
