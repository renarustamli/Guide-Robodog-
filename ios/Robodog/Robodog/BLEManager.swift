import Combine
import CoreBluetooth
import Foundation

/// Connects to the handheld board ("Robodog") over Bluetooth and acts as its host:
/// sends PING every 0.4 s (the handheld stops after 1.2 s without one), forwards text commands
/// (CAM:*, NAV:*, RECORD, VOICE:*, STOP), and receives status lines and voice audio.
final class BLEManager: NSObject, ObservableObject {
    // Nordic UART Service plus an audio characteristic, matching two-board/src/handheld/ble_link.h
    static let serviceUUID = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    static let rxUUID = CBUUID(string: "6E400002-B5A3-F393-E0A9-E50E24DCCA9E")     // phone -> board
    static let txUUID = CBUUID(string: "6E400003-B5A3-F393-E0A9-E50E24DCCA9E")     // board -> phone, text
    static let audioUUID = CBUUID(string: "6E400004-B5A3-F393-E0A9-E50E24DCCA9E")  // board -> phone, PCM

    struct HandheldState {
        var epoch: UInt32 = 0
        var speed = 0
        var direction = "stop"
        var stopHeld = false
    }

    @Published var state = "Bluetooth starting…"
    @Published var isConnected = false
    @Published var handheld: HandheldState?
    /// Latest status line other than the twice-a-second STATE line.
    @Published var boardStatus = "No board status yet."

    /// Every text line from the board, on the main thread.
    var onLine: ((String) -> Void)?
    /// Raw audio chunks between AUDIO_BEGIN and AUDIO_END, on the main thread.
    var onAudio: ((Data) -> Void)?

    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var rx: CBCharacteristic?
    private var heartbeat: Timer?

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    /// Call on the main thread.
    func send(_ command: String) {
        guard let peripheral, let rx, let data = (command + "\n").data(using: .utf8) else { return }
        // Heartbeats and camera frames repeat every few hundred ms, so losing one is fine.
        // Everything else (voice, navigation, stop) must arrive.
        let frequent = command == "PING" || command.hasPrefix("CAM:")
        peripheral.writeValue(data, for: rx, type: frequent ? .withoutResponse : .withResponse)
    }

    private func scan() {
        state = "Searching for Robodog…"
        central.scanForPeripherals(withServices: [Self.serviceUUID])
    }

    private func reset() {
        heartbeat?.invalidate()
        heartbeat = nil
        peripheral = nil
        rx = nil
        isConnected = false
        handheld = nil
        if central.state == .poweredOn { scan() }
    }

    private func startHeartbeat() {
        heartbeat?.invalidate()
        heartbeat = Timer.scheduledTimer(withTimeInterval: 0.4, repeats: true) { [weak self] _ in
            self?.send("PING")
        }
        send("PING")
    }

    private func handle(line: String) {
        if line.hasPrefix("STATE:") {
            // STATE:<epoch>,<speed>,<direction>,<stop held 0/1>
            let fields = line.dropFirst(6).split(separator: ",")
            if fields.count == 4 {
                handheld = HandheldState(epoch: UInt32(fields[0]) ?? 0, speed: Int(fields[1]) ?? 0,
                                         direction: String(fields[2]), stopHeld: fields[3] == "1")
            }
        } else {
            boardStatus = line
        }
        onLine?(line)
    }
}

extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        if central.state == .poweredOn {
            scan()
        } else {
            state = "Bluetooth unavailable. Turn it on and allow access in Settings."
            isConnected = false
        }
    }

    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        central.stopScan()
        self.peripheral = peripheral
        peripheral.delegate = self
        state = "Connecting to Robodog…"
        central.connect(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        peripheral.discoverServices([Self.serviceUUID])
    }

    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        reset()
    }

    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        reset()
    }
}

extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        for service in peripheral.services ?? [] where service.uuid == Self.serviceUUID {
            peripheral.discoverCharacteristics([Self.rxUUID, Self.txUUID, Self.audioUUID], for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for characteristic in service.characteristics ?? [] {
            switch characteristic.uuid {
            case Self.rxUUID: rx = characteristic
            case Self.txUUID, Self.audioUUID: peripheral.setNotifyValue(true, for: characteristic)
            default: break
            }
        }
        if rx != nil {
            isConnected = true
            state = "Connected to Robodog"
            startHeartbeat()
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value else { return }
        if characteristic.uuid == Self.audioUUID {
            onAudio?(data)
        } else if let text = String(data: data, encoding: .utf8) {
            handle(line: text.trimmingCharacters(in: .whitespacesAndNewlines))
        }
    }
}
