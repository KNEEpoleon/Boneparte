//
//  TCPClient.swift
//  SVD_ROS_Comms
//
//  TCP client for control commands (Port 5000)
//

import Foundation
import Network
import SwiftUI

class TCPClient: ObservableObject {
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "TCP Client Queue")

    @Published var statusMessage: String = "Waiting for connection..."
    @Published var statusColor: Color = .gray
    @Published var isConnected: Bool = false

    private var host: String
    private var port: UInt16
    private var receivedDataBuffer: String = ""

    init(host: String, port: UInt16) {
        self.host = host
        self.port = port
    }

    func connect() {
        connection = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!, using: .tcp)

        connection?.stateUpdateHandler = { [weak self] newState in
            DispatchQueue.main.async {
                switch newState {
                case .ready:
                    self?.statusMessage = "Connected to server"
                    self?.statusColor = .green
                    self?.isConnected = true
                case .failed(let error):
                    self?.statusMessage = "Connection failed: \(error.localizedDescription)"
                    self?.statusColor = .red
                    self?.isConnected = false
                    self?.attemptReconnect()
                case .cancelled:
                    self?.statusMessage = "Connection cancelled"
                    self?.statusColor = .red
                    self?.isConnected = false
                case .waiting(let error):
                    self?.statusMessage = "Waiting..."
                    self?.statusColor = .orange
                    self?.attemptReconnect()
                default:
                    break
                }
            }
        }

        connection?.start(queue: queue)
        receive()
    }

    func send(_ message: String) {
        guard let connection = connection, connection.state == .ready else {
            DispatchQueue.main.async {
                self.statusMessage = "Connection not ready"
                self.statusColor = .red
            }
            return
        }

        let data = Data(message.utf8)
        connection.send(content: data, completion: .contentProcessed({ [weak self] error in
            DispatchQueue.main.async {
                if let error = error {
                    self?.statusMessage = "Send error: \(error.localizedDescription)"
                    self?.statusColor = .red
                } else {
                    self?.statusMessage = "Sent: \(message.trimmingCharacters(in: .whitespacesAndNewlines))"
                    self?.statusColor = .green
                }
            }
        }))
    }

    private func receive() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self] data, _, isComplete, error in
            if let data = data, !data.isEmpty {
                let newData = String(decoding: data, as: UTF8.self)
                self?.receivedDataBuffer.append(newData)
                self?.processBufferedData()
            }
            if let error = error {
                print("TCP Receive error: \(error.localizedDescription)")
                DispatchQueue.main.async {
                    self?.statusMessage = "Receive error: \(error.localizedDescription)"
                    self?.statusColor = .red
                    self?.isConnected = false
                    self?.attemptReconnect()
                }
            }
            if isComplete {
                print("Connection closed by server")
                DispatchQueue.main.async {
                    self?.statusMessage = "Connection closed by server"
                    self?.statusColor = .red
                    self?.isConnected = false
                    self?.attemptReconnect()
                }
            } else if error == nil {
                self?.receive()
            }
        }
    }
    
    private func processBufferedData() {
        // Look for complete messages (ending with newline)
        while let newlineIndex = receivedDataBuffer.firstIndex(of: "\n") {
            let message = String(receivedDataBuffer[..<newlineIndex])
            let distance = receivedDataBuffer.distance(from: receivedDataBuffer.startIndex, to: newlineIndex) + 1
            receivedDataBuffer.removeFirst(distance)
            
            // Handle responses (acknowledgments, etc.)
            print("Control channel received: \(message)")
        }
    }
    
    private func attemptReconnect() {
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) { [weak self] in
            guard let self = self, !self.isConnected else { return }
            print("Attempting to reconnect (Control)...")
            self.connect()
        }
    }

    func disconnect() {
        connection?.cancel()
        connection = nil
        DispatchQueue.main.async {
            self.statusMessage = "Disconnected"
            self.statusColor = .red
            self.isConnected = false
        }
    }
}

// Annotation data structure
struct AnnotationPoint: Codable {
    let x: Double // Normalized x coordinate (0.0 to 1.0)
    let y: Double // Normalized y coordinate (0.0 to 1.0)
}
