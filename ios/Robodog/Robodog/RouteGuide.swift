import Combine
import CoreLocation
import MapKit

/// Finds a walking route with Apple Maps and sends a turn cue (LEFT / RIGHT / STRAIGHT)
/// when the user gets close to each turn, then ARRIVED at the destination.
final class RouteGuide: NSObject, ObservableObject {
    static let cueDistance: CLLocationDistance = 15  // metres before a turn

    // Indoor demo for the hackathon table, where GPS won't move. Same as the browser demo.
    static let demoRoute: [(cue: String, text: String, wait: TimeInterval)] = [
        ("STRAIGHT", "Walk straight for 20 m", 4),
        ("LEFT", "Turn left onto the high street", 6),
        ("STRAIGHT", "Continue straight for 50 m", 5),
        ("RIGHT", "Turn right at the corner", 6),
        ("ARRIVED", "You have arrived at Tesco", 0),
    ]

    @Published var instruction = "No route."
    @Published var isGuiding = false
    /// Called on the main thread with LEFT, RIGHT, STRAIGHT or ARRIVED.
    var onCue: ((String) -> Void)?

    private let location = CLLocationManager()
    private var steps: [MKRoute.Step] = []
    private var nextStep = 0
    private var destination: CLLocation?
    private var generation = 0

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
                    return
                }
                self.begin(route, name: place.name ?? query)
            }
        }
    }

    func playDemo() {
        stop()
        isGuiding = true
        demoStep(0, generation)
    }

    func stop() {
        generation += 1
        location.stopUpdatingLocation()
        steps = []
        destination = nil
        isGuiding = false
        instruction = "No route."
    }

    private func begin(_ route: MKRoute, name: String) {
        steps = route.steps
        nextStep = 1  // step 0 is the starting point
        let points = route.polyline.points()
        destination = CLLocation(points[route.polyline.pointCount - 1].coordinate)
        isGuiding = true
        instruction = "Route to \(name): \(Int(route.distance)) m."
        onCue?("STRAIGHT")
        location.startUpdatingLocation()
    }

    private func demoStep(_ index: Int, _ current: Int) {
        guard current == generation else { return }
        let step = Self.demoRoute[index]
        onCue?(step.cue)
        instruction = "Demo \(index + 1)/\(Self.demoRoute.count): \(step.text)"
        if index + 1 < Self.demoRoute.count {
            DispatchQueue.main.asyncAfter(deadline: .now() + step.wait) { self.demoStep(index + 1, current) }
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
