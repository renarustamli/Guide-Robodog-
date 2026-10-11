import Combine
import Foundation

/// Drives the rover along a short scripted route (dummy data for now, no GPS or maps).
/// "I want to go to the supermarket" -> 5 m forward, turn right, turn left, 3 m forward.
///
/// Each leg is a drive word (VOICE:<epoch>:forward / left / right) held for a calculated time,
/// then VOICE:<epoch>:stop. The epoch comes from the board's STATE lines; every stop bumps it,
/// so the next leg waits for the stop's STATE before it is sent. The route aborts at once on a
/// physical stop, a camera refusal, an error from the board, a lost connection, or Stop.
final class RouteExecutor: ObservableObject {
    enum Leg {
        case forward(metres: Double)
        case left(degrees: Double)
        case right(degrees: Double)

        var word: String {
            switch self {
            case .forward: return "forward"
            case .left: return "left"
            case .right: return "right"
            }
        }

        var spoken: String {
            switch self {
            case .forward(let metres): return "\(Self.format(metres)) \(metres == 1 ? "metre" : "metres") forward"
            case .left: return "turn left"
            case .right: return "turn right"
            }
        }

        private static func format(_ value: Double) -> String {
            value == value.rounded() ? String(Int(value)) : String(format: "%.1f", value)
        }
    }

    struct Route {
        let name: String
        let legs: [Leg]
    }

    // Drive times, measured on the rover: 2.5 s per metre forward, 0.01 s per degree of turn.
    // Re-measure if the battery, floor or wheels change.
    static let secondsPerMetre = 2.5
    static let secondsPerDegree = 0.01
    /// Pause between legs.
    static let legPause: TimeInterval = 0.8
    /// Pause after announcing the route, so the user can step back.
    static let startDelay: TimeInterval = 2.5

    /// Dummy destinations. Keywords are matched as whole words in what the user said.
    static let dummyRoutes: [(keywords: [String], route: Route)] = [
        (["supermarket", "tesco", "shop", "store", "groceries"],
         Route(name: "the supermarket", legs: [.forward(metres: 5), .right(degrees: 90), .left(degrees: 90), .forward(metres: 3)])),
        (["museum", "gallery", "exhibition"],
         Route(name: "the museum", legs: [.forward(metres: 3), .left(degrees: 90), .forward(metres: 2)])),
    ]

    @Published var status = "No route."
    @Published var isRunning = false

    /// Sends a command to the board. Set by the view.
    var send: ((String) -> Void)?
    /// Speaks a sentence. Set by the view.
    var say: ((String) -> Void)?

    private enum Phase { case idle, starting, waitingForMove, moving, waitingForStop }
    private var phase = Phase.idle
    private var route: Route?
    private var index = 0
    private var epoch: UInt32 = 0
    private var generation = 0
    private var timeout: DispatchWorkItem?
    /// Ends the current forward/turn leg. Paused while the board goes around an obstacle.
    private var legTimer: DispatchWorkItem?
    private var legEndsAt = Date()
    private var legRemaining: TimeInterval?

    /// The scripted route for a place name, or nil if it isn't one of the dummy places.
    static func route(for place: String) -> Route? {
        let words = Set(place.lowercased().split(whereSeparator: { !$0.isLetter }).map(String.init))
        return dummyRoutes.first { !words.isDisjoint(with: $0.keywords) }?.route
    }

    func start(_ route: Route) {
        cancel(announce: nil)
        generation += 1
        self.route = route
        index = 0
        isRunning = true
        phase = .starting
        let plan = route.legs.map(\.spoken).joined(separator: ", then ")
        status = "Route to \(route.name): \(plan)."
        say?("Going to \(route.name): \(plan). Starting in a moment.")
        let current = generation
        DispatchQueue.main.asyncAfter(deadline: .now() + Self.startDelay) { [weak self] in
            guard let self, current == self.generation else { return }
            self.beginLeg()
        }
    }

    /// Stops the rover and abandons the route. `announce` is spoken if given.
    func cancel(announce: String? = "Route cancelled.", sendStop: Bool = true) {
        guard isRunning else { return }
        generation += 1
        timeout?.cancel()
        legTimer?.cancel()
        legRemaining = nil
        isRunning = false
        phase = .idle
        route = nil
        status = "No route."
        if sendStop { send?("STOP") }
        if let announce { say?(announce) }
    }

    /// Every text line from the board.
    func handle(line: String) {
        if line.hasPrefix("STATE:") {
            // STATE:<epoch>,<speed>,<direction>,<stop held>
            let fields = line.dropFirst(6).split(separator: ",")
            guard fields.count == 4, let newEpoch = UInt32(fields[0]) else { return }
            epoch = newEpoch
            guard isRunning, let route else { return }
            let direction = String(fields[2])
            switch phase {
            case .waitingForMove where direction == route.legs[index].word:
                timeout?.cancel()
                phase = .moving
                scheduleLegEnd(after: Self.duration(of: route.legs[index]))
            case .waitingForStop where direction == "stop":
                timeout?.cancel()
                index += 1
                let current = generation
                DispatchQueue.main.asyncAfter(deadline: .now() + Self.legPause) { [weak self] in
                    guard let self, current == self.generation else { return }
                    self.beginLeg()
                }
            default:
                break
            }
        } else if isRunning, phase == .moving, line == "AVOID:LEFT" || line == "AVOID:RIGHT" {
            // The board is detouring around an obstacle: pause the leg so the detour doesn't use it up.
            legTimer?.cancel()
            legRemaining = max(0, legEndsAt.timeIntervalSinceNow)
            status = "Going around an obstacle…"
        } else if isRunning, phase == .moving, line == "AVOID:Done", let remaining = legRemaining {
            legRemaining = nil
            status = "Leg \(index + 1) of \(route?.legs.count ?? 0): \(route?.legs[index].spoken ?? "")"
            scheduleLegEnd(after: remaining)
        } else if isRunning, line.hasPrefix("REFUSED:") {
            cancel(announce: "I can't go forward. The path ahead isn't clear. Route cancelled.")
        } else if isRunning, line.hasPrefix("STOP:"), phase != .waitingForStop {
            // A stop we didn't ask for: physical button, camera obstacle, lost heartbeat.
            cancel(announce: "Stopped. Route cancelled.", sendStop: false)
        } else if isRunning, line.hasPrefix("ERROR:") {
            cancel(announce: "The robot refused the command. Route cancelled.")
        }
    }

    private static func duration(of leg: Leg) -> TimeInterval {
        switch leg {
        case .forward(let metres): return metres * secondsPerMetre
        case .left(let degrees), .right(let degrees): return degrees * secondsPerDegree
        }
    }

    private func beginLeg() {
        guard let route, isRunning else { return }
        guard index < route.legs.count else {
            isRunning = false
            phase = .idle
            self.route = nil
            status = "Arrived at \(route.name)."
            say?("You have arrived at \(route.name).")
            return
        }
        let leg = route.legs[index]
        status = "Leg \(index + 1) of \(route.legs.count): \(leg.spoken)"
        phase = .waitingForMove
        send?("VOICE:\(epoch):\(leg.word)")
        armTimeout("The robot didn't respond. Route cancelled.")
    }

    private func scheduleLegEnd(after seconds: TimeInterval) {
        legTimer?.cancel()
        legEndsAt = Date().addingTimeInterval(seconds)
        let current = generation
        let item = DispatchWorkItem { [weak self] in
            guard let self, current == self.generation, self.phase == .moving else { return }
            self.endLeg()
        }
        legTimer = item
        DispatchQueue.main.asyncAfter(deadline: .now() + seconds, execute: item)
    }

    private func endLeg() {
        phase = .waitingForStop
        send?("VOICE:\(epoch):stop")
        armTimeout("The robot didn't confirm stopping. Route cancelled.")
    }

    /// Abort if the board doesn't answer a command with a STATE line in time.
    private func armTimeout(_ message: String) {
        timeout?.cancel()
        let item = DispatchWorkItem { [weak self] in self?.cancel(announce: message) }
        timeout = item
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.5, execute: item)
    }
}
