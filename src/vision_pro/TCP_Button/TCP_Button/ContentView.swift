import SwiftUI
import Network
import RealityKit

struct ContentView: View {
    @StateObject private var tcpClient = TCPClient(host: "192.168.0.193", port: 5000)
    @State private var showTrackingUI = false
    @StateObject private var trackingAppState = AppState()

    
    var body: some View {
        VStack(spacing: 20) {
            
            GroupBox(label: Text("ProSight")) {
                Button("Start/Stop Tracking") {
                    showTrackingUI.toggle()
                }
                .padding()
                .frame(minWidth: 100)
                .sheet(isPresented: $showTrackingUI) {
                    HomeContentView(immersiveSpaceIdentifier: "ObjectTracking", appState: trackingAppState)
                }
            }

            // ParaSight Section
            GroupBox(label: Text("ParaSight")) {
                HStack(spacing: 10) {
                    ForEach(["Annotate", "Re-Register", "Reset", "Start"], id: \.self) { label in
                        Button(label) {
                            tcpClient.send("\(label)\n")
                        }
                        .padding()
                        .frame(minWidth: 80)
                    }
                }
            }

            // BONEparte Section
            GroupBox(label: Text("BONEparte")) {
                HStack(spacing: 10) {
                    ForEach(["Drill", "Track", "Reset", "Stop"], id: \.self) { label in
                        Button(label) {
                            tcpClient.send("\(label)\n")
                        }
                        .padding()
                        .frame(minWidth: 80)
                    }
                }
            }

            // Poses Section
            GroupBox(label: Text("Poses")) {
                HStack(spacing: 10) {
                    ForEach(["Femur 1", "Femur 2", "Femur 3", "Tibia 1", "Tibia 2"], id: \.self) { label in
                        Button(label) {
                            tcpClient.send("\(label)\n")
                        }
                        .padding()
                        .frame(minWidth: 100)
                    }
                }
            }
            
            // Connection status indicator
            Text("Connection: \(tcpClient.connectionState)")
                .foregroundColor(tcpClient.connectionState == "Connected" ? .green : .red)
                .padding(.top)
        }
        .padding()
        .onAppear {
            tcpClient.connect()
            
            // Request required permissions for tracking
            Task {
                if trackingAppState.allRequiredProvidersAreSupported {
                    await trackingAppState.requestWorldSensingAuthorization()
                }
            }
        }
        .onDisappear {
            tcpClient.disconnect()
        }
    }
}

class TCPClient: ObservableObject {
    private var connection: NWConnection
    private let queue = DispatchQueue(label: "TCP Client Queue")
    @Published var connectionState: String = "Disconnected"
    
    init(host: String, port: UInt16) {
        self.connection = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!, using: .tcp)
    }

    func connect() {
        connectionState = "Connecting..."
        connection.stateUpdateHandler = { [weak self] newState in
            DispatchQueue.main.async {
                switch newState {
                case .ready:
                    print("TCP connection ready")
                    self?.connectionState = "Connected"
                case .failed(let error):
                    print("TCP connection failed: \(error)")
                    self?.connectionState = "Failed: \(error.localizedDescription)"
                    // Try to reconnect after a delay
                    DispatchQueue.main.asyncAfter(deadline: .now() + 3) {
                        self?.reconnect()
                    }
                case .cancelled:
                    print("TCP connection cancelled")
                    self?.connectionState = "Disconnected"
                case .preparing:
                    self?.connectionState = "Preparing..."
                case .waiting(let error):
                    print("Connection waiting: \(error)")
                    self?.connectionState = "Waiting to connect..."
                default:
                    break
                }
            }
        }
        connection.start(queue: queue)
        receive()
    }
    
    private func reconnect() {
        // Create a new connection with the same parameters
        let host = connection.endpoint.debugDescription.components(separatedBy: ":").first ?? "unknown"
        let port = UInt16(connection.endpoint.debugDescription.components(separatedBy: ":").last ?? "0") ?? 5000
        
        connection.cancel()
        connection = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!, using: .tcp)
        connect()
    }

    func send(_ message: String) {
        guard connection.state == .ready else {
            print("Connection not ready")
            return
        }
        let data = Data(message.utf8)
        connection.send(content: data, completion: .contentProcessed({ error in
            if let error = error {
                print("Send error: \(error.localizedDescription)")
            }
        }))
    }

    private func receive() {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 1024) { [weak self] data, _, isComplete, error in
            if let data = data, !data.isEmpty {
                let response = String(decoding: data, as: UTF8.self)
                DispatchQueue.main.async {
                    print("Received from server: \(response.trimmingCharacters(in: .whitespacesAndNewlines))")
                }
            }
            if let error = error {
                print("Receive error: \(error.localizedDescription)")
            }
            if isComplete {
                print("Connection closed by server")
                DispatchQueue.main.async {
                    self?.connectionState = "Disconnected by server"
                }
            } else if error == nil {
                self?.receive()
            }
        }
    }

    func disconnect() {
        connection.cancel()
        connectionState = "Disconnected"
    }
}
