//
//  ImageTCPClient.swift
//  SVD_ROS_Comms
//
//  TCP client for sending/receiving images and annotations (Port 5002)
//

import Foundation
import Network
import SwiftUI

class ImageTCPClient: ObservableObject {
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "ImageTCPClient")
    
    @Published var statusMessage: String = "Waiting for connection..."
    @Published var statusColor: Color = .gray
    @Published var isConnected: Bool = false
    @Published var receivedImage: Data?
    @Published var receivedSegmentedImage: Data?
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
                    self?.statusMessage = "Connected (Images)"
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
    
    func disconnect() {
        connection?.cancel()
        connection = nil
        DispatchQueue.main.async {
            self.statusMessage = "Disconnected"
            self.statusColor = .red
            self.isConnected = false
        }
    }
    
    private func attemptReconnect() {
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) { [weak self] in
            guard let self = self, !self.isConnected else { return }
            print("Attempting to reconnect (Images)...")
            self.connect()
        }
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
                    self?.statusMessage = "Sent: \(message.prefix(50))..."
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
                print("TCP Receive error (Images): \(error.localizedDescription)")
                DispatchQueue.main.async {
                    self?.statusMessage = "Receive error: \(error.localizedDescription)"
                    self?.statusColor = .red
                    self?.isConnected = false
                    self?.attemptReconnect()
                }
            }
            if isComplete {
                print("Connection closed by server (Images)")
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
            
            if message.hasPrefix("IMAGE:") {
                handleImageData(message, isSegmented: false)
            } else if message.hasPrefix("SEGMENTED_IMAGE:") {
                handleImageData(message, isSegmented: true)
            }
        }
    }
    
    private func handleImageData(_ response: String, isSegmented: Bool) {
        let prefixLength = isSegmented ? 16 : 6 // "SEGMENTED_IMAGE:" (16 chars) or "IMAGE:" (6 chars)
        let imageDataString = String(response.dropFirst(prefixLength))
        
        if let imageData = Data(base64Encoded: imageDataString, options: .ignoreUnknownCharacters) {
            DispatchQueue.main.async {
                if isSegmented {
                    self.receivedSegmentedImage = imageData
                } else {
                    self.receivedImage = imageData
                }
                self.imageTransmissionStatus = "Image received successfully (\(imageData.count) bytes)"
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
            
            print("Sending \(annotations.count) annotations")
            
            let data = Data(message.utf8)
            connection.send(content: data, completion: .contentProcessed({ [weak self] error in
                DispatchQueue.main.async {
                    if let error = error {
                        self?.statusMessage = "Annotation send error: \(error.localizedDescription)"
                        self?.statusColor = .red
                    } else {
                        self?.statusMessage = "Annotations sent successfully"
                        self?.statusColor = .green
                        self?.receivedImage = nil
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
}

