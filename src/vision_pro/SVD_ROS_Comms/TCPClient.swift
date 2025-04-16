//
//  TCPClient.swift
//  SVD_ROS_Comms
//
//  Created by Kneepoleon Boneparte on 4/16/25.
//
import Foundation
import Network
import SwiftUI

class TCPClient: ObservableObject {
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "TCP Client Queue")

    @Published var statusMessage: String = "Waiting for connection..."
    @Published var statusColor: Color = .gray

    private var host: String
    private var port: UInt16

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
                case .failed(let error):
                    self?.statusMessage = "Connection failed: \(error.localizedDescription)"
                    self?.statusColor = .red
                case .cancelled:
                    self?.statusMessage = "Connection cancelled"
                    self?.statusColor = .red
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
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 1024) { [weak self] data, _, isComplete, error in
            if let data = data, !data.isEmpty {
                let response = String(decoding: data, as: UTF8.self)
                print("Received from server: \(response.trimmingCharacters(in: .whitespacesAndNewlines))")
            }
            if let error = error {
                print("Receive error: \(error.localizedDescription)")
            }
            if isComplete {
                print("Connection closed by server")
            } else if error == nil {
                self?.receive()
            }
        }
    }

    func disconnect() {
        connection?.cancel()
        connection = nil
    }
}

