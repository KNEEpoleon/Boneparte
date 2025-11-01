//
//  TCPClient.swift
//  ArucoTransform
//
//  TCP client for receiving drill poses from ROS server
//

import Foundation
import Network

class TCPClient: ObservableObject {
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "TCPClient")
    
    private var host: String
    private var port: UInt16
    
    var onDrillPosesReceived: (([DrillSite]) -> Void)?
    var onConnectionStateChanged: ((Bool, String) -> Void)?
    
    init(host: String, port: UInt16) {
        self.host = host
        self.port = port
    }
    
    func connect() {
        let nwHost = NWEndpoint.Host(host)
        let nwPort = NWEndpoint.Port(rawValue: port)!
        
        connection = NWConnection(host: nwHost, port: nwPort, using: .tcp)
        
        connection?.stateUpdateHandler = { [weak self] newState in
            DispatchQueue.main.async {
                switch newState {
                case .ready:
                    self?.onConnectionStateChanged?(true, "Connected to \(self?.host ?? ""):\(self?.port ?? 0)")
                    print("✅ TCP connected to \(self?.host ?? ""):\(self?.port ?? 0)")
                case .failed(let error):
                    self?.onConnectionStateChanged?(false, "Failed: \(error.localizedDescription)")
                    print("❌ TCP connection failed: \(error)")
                case .cancelled:
                    self?.onConnectionStateChanged?(false, "Connection cancelled")
                    print("TCP connection cancelled")
                default:
                    break
                }
            }
        }
        
        connection?.start(queue: queue)
        receive()
    }
    
    func disconnect() {
        connection?.cancel()
        connection = nil
        onConnectionStateChanged?(false, "Disconnected")
    }
    
    private func receive() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self] data, _, isComplete, error in
            if let data = data, !data.isEmpty {
                let message = String(decoding: data, as: UTF8.self)
                self?.parseMessage(message)
            }
            
            if let error = error {
                print("Receive error: \(error)")
                DispatchQueue.main.async {
                    self?.onConnectionStateChanged?(false, "Error: \(error.localizedDescription)")
                }
            }
            
            if isComplete {
                print("Connection closed by server")
                DispatchQueue.main.async {
                    self?.onConnectionStateChanged?(false, "Server closed connection")
                }
            } else if error == nil {
                // Continue receiving
                self?.receive()
            }
        }
    }
    
    private func parseMessage(_ message: String) {
        // Expected format: POSES|x,y,z,qx,qy,qz,qw|x,y,z,qx,qy,qz,qw|...
        let lines = message.split(separator: "\n")
        
        for line in lines {
            let trimmed = line.trimmingCharacters(in: .whitespacesAndNewlines)
            guard trimmed.hasPrefix("POSES|") else { continue }
            
            let poseString = String(trimmed.dropFirst(6))  // Remove "POSES|"
            let poseStrings = poseString.split(separator: "|")
            
            var drillSites: [DrillSite] = []
            
            for poseStr in poseStrings {
                let components = poseStr.split(separator: ",").compactMap { Float($0) }
                guard components.count == 7 else { continue }
                
                let position = SIMD3<Float>(components[0], components[1], components[2])
                let orientation = simd_quatf(
                    ix: components[3],
                    iy: components[4],
                    iz: components[5],
                    r: components[6]
                )
                
                drillSites.append(DrillSite(position: position, orientation: orientation))
            }
            
            if !drillSites.isEmpty {
                DispatchQueue.main.async { [weak self] in
                    self?.onDrillPosesReceived?(drillSites)
                }
                print("📍 Received \(drillSites.count) drill sites from ROS")
            }
        }
    }
    
    func send(_ message: String) {
        guard let connection = connection, connection.state == .ready else {
            print("Cannot send: connection not ready")
            return
        }
        
        let data = Data(message.utf8)
        connection.send(content: data, completion: .contentProcessed { error in
            if let error = error {
                print("Send error: \(error)")
            }
        })
    }
}

