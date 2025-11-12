
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
    @Published var receivedImage: Data?
    @Published var imageTransmissionStatus: String = ""

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
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 1024 * 1024 * 10) { [weak self] data, _, isComplete, error in
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
                }
            }
            if isComplete {
                print("Connection closed by server")
                DispatchQueue.main.async {
                    self?.statusMessage = "Connection closed by server"
                    self?.statusColor = .red
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
            
            if message.hasPrefix("IMAGE:") {
                handleImageData(message)
            } else {
                // Only print non-image messages (commands, status, etc.)
                print("Received command: \(message)")
            }
        }
    }
    
    private func handleImageData(_ response: String) {
        let imageDataString = String(response.dropFirst(6)) // Remove "IMAGE:" prefix
        
        // Validate base64 string length (should be reasonable)
        guard imageDataString.count > 100 else {
            print("Error: Base64 image data too short (\(imageDataString.count) characters)")
            DispatchQueue.main.async {
                self.imageTransmissionStatus = "Error: Invalid image data length"
            }
            return
        }
        
        if let imageData = Data(base64Encoded: imageDataString) {
            DispatchQueue.main.async {
                self.receivedImage = imageData
                self.imageTransmissionStatus = "Image received successfully (\(imageData.count) bytes)"
                print("Image decoded successfully: \(imageData.count) bytes")
            }
        } else {
            print("Error: Failed to decode base64 image data (length: \(imageDataString.count))")
            DispatchQueue.main.async {
                self.imageTransmissionStatus = "Error: Failed to decode image data"
            }
        }
    }

    func sendAnnotations(_ annotations: [AnnotationPoint]) {
        guard let connection = connection, connection.state == .ready else {
            DispatchQueue.main.async {
                self.statusMessage = "Connection not ready for annotation transmission"
                self.statusColor = .red
            }
            return
        }
        
        do {
            let annotationData = try JSONEncoder().encode(annotations)
            let annotationString = String(data: annotationData, encoding: .utf8) ?? ""
            let message = "ANNOTATIONS:\(annotationString)\n"
            
            // Debug: Print what we're sending
            print("=== DEBUG: Sending annotations ===")
            print("Number of annotations: \(annotations.count)")
            print("Annotation data: \(annotations)")
            print("JSON string: \(annotationString)")
            print("Full message: \(message)")
            print("Message length: \(message.count) characters")
            print("================================")
            
            let data = Data(message.utf8)
            connection.send(content: data, completion: .contentProcessed({ [weak self] error in
                DispatchQueue.main.async {
                    if let error = error {
                        self?.statusMessage = "Annotation send error: \(error.localizedDescription)"
                        self?.statusColor = .red
                    } else {
                        self?.statusMessage = "Annotations sent successfully"
                        self?.statusColor = .green
                        self?.receivedImage = nil // Clear image after sending annotations
                    }
                }
            }))
        } catch {
            DispatchQueue.main.async {
                self.statusMessage = "Failed to encode annotations: \(error.localizedDescription)"
                self.statusColor = .red
            }
        }
    }

    func disconnect() {
        connection?.cancel()
        connection = nil
    }
}

// Annotation data structure
struct AnnotationPoint: Codable {
    let x: Double // Normalized x coordinate (0.0 to 1.0)
    let y: Double // Normalized y coordinate (0.0 to 1.0)
}
