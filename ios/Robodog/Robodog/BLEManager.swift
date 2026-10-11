import Combine
import CoreBluetooth
import Foundation

/// Connects to the Genesis Mini ("Robodog") over Bluetooth and sends the same text
/// commands the USB Camera Test uses: CAM:CLEAR, CAM:BLOCK, CAM:OFF, NAV:<cue>.
final class BLEManager: NSObject, ObservableObject {
    // Nordic UART Service, matching firmware/ble.h
    static let serviceUUID = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    static let rxUUID = CBUUID(string: "6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  // phone -> board
    static let txUUID = CBUUID(string: "6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  // board -> phone

    @Published var state = "Bluetooth starting…"
    @Published var isConnected = false
    @Published var boardStatus = "No board status yet."

    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var rx: CBCharacteristic?

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: nil)
    }

    /// Call on the main thread.
    func send(_ command: String) {
        guard let peripheral, let rx, let data = (command + "\n").data(using: .utf8) else { return }
        // Camera frames arrive every 200 ms, so losing one is fine. Navigation cues must arrive.
        let type: CBCharacteristicWriteType = command.hasPrefix("CAM:") ? .withoutResponse : .withResponse
        peripheral.writeValue(data, for: rx, type: type)
    }

    private func scan() {
        state = "Searching for Robodog…"
        central.scanForPeripherals(withServices: [Self.serviceUUID])
    }

    private func reset() {
        peripheral = nil
        rx = nil
        isConnected = false
        if central.state == .poweredOn { scan() }
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
            peripheral.discoverCharacteristics([Self.rxUUID, Self.txUUID], for: service)
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        for characteristic in service.characteristics ?? [] {
            if characteristic.uuid == Self.rxUUID { rx = characteristic }
            if characteristic.uuid == Self.txUUID { peripheral.setNotifyValue(true, for: characteristic) }
        }
        if rx != nil {
            isConnected = true
            state = "Connected to Robodog"
        }
    }

    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value, let text = String(data: data, encoding: .utf8) else { return }
        boardStatus = text.trimmingCharacters(in: .whitespacesAndNewlines)
    }
}
