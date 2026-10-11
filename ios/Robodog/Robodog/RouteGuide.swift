import Combine
import CoreLocation
import MapKit

/// Finds a walking route with Apple Maps and sends a turn cue (LEFT / RIGHT / STRAIGHT)
/// when the user gets close to each turn, then ARRIVED at the destination.
final class RouteGuide: NSObject, ObservableObject {
    static let cueDistance: CLLocationDistance = 15  // metres before a turn

    // Indoor demos for the hackathon table, where GPS won't move. Same routes as the browser demo.
    typealias DemoStep = (cue: String, text: String, wait: TimeInterval)
    static let demoRoutes: [(name: String, steps: [DemoStep])] = [
        ("Dairy fridge", [
            ("STRAIGHT", "Walk in through the main doors, straight for 5 m", 4),
            ("RIGHT", "Turn right into fruit and veg", 5),
            ("STRAIGHT", "Continue straight past the produce for 4 m", 4),
            ("LEFT", "Turn left into aisle 3, bakery and bread", 5),
            ("STRAIGHT", "Walk down the aisle for 6 m", 5),
            ("RIGHT", "Turn right at the end of the aisle, towards the chilled section", 5),
            ("STRAIGHT", "Continue straight for 3 m", 4),
            ("ARRIVED", "You have arrived at the dairy fridge", 0),
        ]),
        ("Pharmacy", [
            ("STRAIGHT", "Walk in through the main doors, straight for 3 m", 4),
            ("LEFT", "Turn left along the front wall", 5),
            ("STRAIGHT", "Continue straight past customer service for 6 m", 5),
            ("RIGHT", "Turn right at the flower display", 5),
            ("STRAIGHT", "Continue straight for 2 m", 3),
            ("ARRIVED", "You have arrived at the pharmacy counter", 0),
        ]),
        ("Frozen foods", [
            ("STRAIGHT", "Walk in through the main doors, straight for 4 m", 4),
            ("RIGHT", "Turn right into aisle 7, tea and coffee", 5),
            ("STRAIGHT", "Walk down the aisle for 8 m", 6),
            ("LEFT", "Turn left at the end of the aisle", 5),
            ("STRAIGHT", "Continue straight along the back wall for 3 m", 4),
            ("LEFT", "Turn left into the frozen aisle", 5),
            ("STRAIGHT", "Continue straight for 4 m", 4),
            ("ARRIVED", "You have arrived at the frozen foods", 0),
        ]),
        ("Cafe", [
            ("STRAIGHT", "Walk straight for 5 m past the trolley bay", 4),
            ("LEFT", "Turn left past the newspapers", 5),
            ("STRAIGHT", "Continue straight for 4 m", 4),
            ("LEFT", "Turn left at the toilets sign", 5),
            ("STRAIGHT", "Continue straight for 3 m", 4),
            ("RIGHT", "Turn right into the cafe seating area", 5),
            ("STRAIGHT", "Continue straight for 2 m", 3),
            ("ARRIVED", "You have arrived at the cafe counter", 0),
        ]),
        ("Checkout", [
            ("STRAIGHT", "Walk straight out of the aisle for 4 m", 4),
            ("LEFT", "Turn left towards the tills", 5),
            ("STRAIGHT", "Continue straight past the bakery counter for 5 m", 5),
            ("RIGHT", "Turn right into till lane 4", 5),
            ("STRAIGHT", "Continue straight for 2 m", 3),
            ("ARRIVED", "You have arrived at till 4", 0),
        ]),
        ("Exit", [
            ("STRAIGHT", "Walk straight past the tills for 3 m", 4),
            ("RIGHT", "Turn right at the trolley bay", 5),
            ("STRAIGHT", "Continue straight for 5 m", 5),
            ("LEFT", "Turn left through the exit doors", 5),
            ("STRAIGHT", "Continue straight outside for 4 m", 4),
            ("ARRIVED", "You have arrived at the car park entrance", 0),
        ]),
    ]

    @Published var instruction = "No route."
    @Published var isGuiding = false
    /// Called on the main thread with LEFT, RIGHT, STRAIGHT or ARRIVED.
    var onCue: ((String) -> Void)?
    /// Called on the main thread with a sentence worth speaking (route found, not found, arrived).
    var onAnnounce: ((String) -> Void)?

    private let location = CLLocationManager()
    private var steps: [MKRoute.Step] = []
    private var nextStep = 0
    private var destination: CLLocation?
    private var generation = 0
    private var pendingRoute: MKRoute?
    /// Set while a found route is waiting for the user to say yes or no.
    @Published var pendingName: String?

    override init() {
        super.init()
        location.delegate = self
        location.desiredAccuracy = kCLLocationAccuracyBest
        location.requestWhenInUseAuthorization()
    }

    func start(to query: String) {
        stop()
        instruction = "Finding \(query)…"
        let search = MKLocalSearch.Request()
        search.naturalLanguageQuery = query
        if let here = location.location {
            search.region = MKCoordinateRegion(center: here.coordinate, latitudinalMeters: 5000, longitudinalMeters: 5000)
        }
        let current = generation
        MKLocalSearch(request: search).start { response, _ in
            guard current == self.generation else { return }
            guard let place = response?.mapItems.first else {
                self.instruction = "Couldn't find \(query)."
                self.onAnnounce?(self.instruction)
                return
            }
            let request = MKDirections.Request()
            request.source = .forCurrentLocation()
            request.destination = place
            request.transportType = .walking
            MKDirections(request: request).calculate { response, _ in
                guard current == self.generation else { return }
                guard let route = response?.routes.first else {
                    self.instruction = "No walking route to \(place.name ?? query)."
                    self.onAnnounce?(self.instruction)
                    return
                }
                self.propose(route, name: place.name ?? query)
            }
        }
    }

    func playDemo(_ route: Int = 0) {
        stop()
        isGuiding = true
        demoStep(route, 0, generation)
    }

    func stop() {
        generation += 1
        location.stopUpdatingLocation()
        steps = []
        destination = nil
        pendingRoute = nil
        pendingName = nil
        isGuiding = false
        instruction = "No route."
    }

    /// A route has been found; ask the user before guiding, so a wrong first search result isn't followed blindly.
    private func propose(_ route: MKRoute, name: String) {
        pendingRoute = route
        pendingName = name
        let distance = Self.spoken(distance: route.distance)
        instruction = "\(name), \(distance) away. Go there?"
        onAnnounce?("\(name), \(distance) away. Shall I take you there? Say yes or no.")
    }

    func confirmPending() {
        guard let route = pendingRoute, let name = pendingName else { return }
        pendingRoute = nil
        pendingName = nil
        begin(route, name: name)
    }

    func cancelPending() {
        guard pendingRoute != nil else { return }
        pendingRoute = nil
        pendingName = nil
        instruction = "Route cancelled."
        onAnnounce?("Okay, cancelled.")
    }

    private static func spoken(distance metres: CLLocationDistance) -> String {
        if metres < 1000 { return "\(Int((metres / 10).rounded()) * 10) metres" }
        return String(format: "%.1f kilometres", metres / 1000)
    }

    private func begin(_ route: MKRoute, name: String) {
        steps = route.steps
        nextStep = 1  // step 0 is the starting point
        let points = route.polyline.points()
        destination = CLLocation(points[route.polyline.pointCount - 1].coordinate)
        isGuiding = true
        instruction = "Route to \(name): \(Int(route.distance)) m."
        onAnnounce?("Route to \(name) ready, \(Int(route.distance)) metres. Say forward to start.")
        onCue?("STRAIGHT")
        location.startUpdatingLocation()
    }

    private func demoStep(_ route: Int, _ index: Int, _ current: Int) {
        guard current == generation else { return }
        let steps = Self.demoRoutes[route].steps
        let step = steps[index]
        onCue?(step.cue)
        instruction = "Demo \(index + 1)/\(steps.count): \(step.text)"
        if index + 1 < steps.count {
            DispatchQueue.main.asyncAfter(deadline: .now() + step.wait) { self.demoStep(route, index + 1, current) }
        } else {
            isGuiding = false
        }
    }

    /// Turn direction from the change in walking bearing at the start of step `index`.
    /// Uses the route geometry, so it works whatever language the instructions are in.
    private func cue(forStep index: Int) -> String {
        let before = steps[index - 1].polyline, after = steps[index].polyline
        guard before.pointCount >= 2, after.pointCount >= 2 else { return "STRAIGHT" }
        let b = before.points(), a = after.points()
        let incoming = bearing(b[before.pointCount - 2], b[before.pointCount - 1])
        let outgoing = bearing(a[0], a[1])
        var delta = outgoing - incoming
        if delta > 180 { delta -= 360 }
        if delta < -180 { delta += 360 }
        if delta > 30 { return "RIGHT" }
        if delta < -30 { return "LEFT" }
        return "STRAIGHT"
    }

    /// Compass bearing in degrees. MKMapPoint y grows southwards.
    private func bearing(_ from: MKMapPoint, _ to: MKMapPoint) -> Double {
        atan2(to.x - from.x, from.y - to.y) * 180 / .pi
    }
}

extension RouteGuide: CLLocationManagerDelegate {
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard isGuiding, !steps.isEmpty, let here = locations.last, here.horizontalAccuracy >= 0 else { return }

        if let destination, here.distance(from: destination) < Self.cueDistance {
            onCue?("ARRIVED")
            stop()
            instruction = "You have arrived."
            onAnnounce?(instruction)
            return
        }

        while nextStep < steps.count {
            let turnPoint = CLLocation(steps[nextStep].polyline.points()[0].coordinate)
            guard here.distance(from: turnPoint) < Self.cueDistance else { break }
            onCue?(cue(forStep: nextStep))
            nextStep += 1
        }

        if nextStep < steps.count {
            let step = steps[nextStep]
            let distance = here.distance(from: CLLocation(step.polyline.points()[0].coordinate))
            instruction = "In \(Int(distance)) m: \(step.instructions)"
        }
    }
}

private extension CLLocation {
    convenience init(_ coordinate: CLLocationCoordinate2D) {
        self.init(latitude: coordinate.latitude, longitude: coordinate.longitude)
    }
}
