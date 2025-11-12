import SwiftUI
import Network

struct ContentView: View {
    @StateObject private var tcpClient = TCPClient(host: "192.168.0.193", port: 5000)
    @StateObject private var appModel = AppModel()
    @State private var estopState: EStopState = .normal
    @State private var estopTimer: Timer?
    
    enum EStopState {
        case normal
        case confirm
    }

    // Button text to command mapping
    let commandMap: [String: String] = [
        "Annotate": "annotate",
        "Restart": "restart",
        
        "Drill": "CMD_DRILL",
        "Track": "CMD_TRACK",
        "Start": "CMD_START",
        "Stop": "CMD_STOP",
        
        "Femur 1": "drill_femur_1",
        "Femur 2": "drill_femur_2",
        "Femur 3": "drill_femur_3",
        "Tibia 1": "drill_tibia_1",
        "Tibia 2": "drill_tibia_2"
    ]

    var body: some View {
        ZStack {
            VStack(spacing: 20) {
                HStack {
                    Spacer()
                    Button("Connect to ROS") {
                        tcpClient.connect()
                    }
                    .padding(10)
                    .background(Color.green)
                    .foregroundColor(.white)
                    .cornerRadius(8)

                    Button("Disconnect") {
                        tcpClient.disconnect()
                        tcpClient.statusMessage = "Disconnected manually"
                        tcpClient.statusColor = .red
                    }
                    .padding(10)
                    .background(Color.red)
                    .foregroundColor(.white)
                    .cornerRadius(8)
                }

                GroupBox(label: Text("ParaSight")) {
                    HStack(spacing: 10) {
                        ForEach(["Annotate", "Restart"], id: \.self) { label in
                            Button(label) {
                                let cmd = commandMap[label] ?? label
                                tcpClient.send("\(cmd)\n")
                                
                                // Handle annotate button specifically
                                if label == "Annotate" {
                                    appModel.annotationState = .waitingForImage
                                    appModel.currentImageData = nil
                                    appModel.currentAnnotations = []
                                }
                            }
                            .disabled(tcpClient.statusColor != .green)
                            .opacity(tcpClient.statusColor == .green ? 1.0 : 0.5)
                            .padding()
                            .frame(minWidth: 80)
                        }
                    }
                }

                GroupBox(label: Text("BONEparte")) {
                    HStack(spacing: 10) {
                        ForEach(["Drill", "Track", "Start", "Stop"], id: \.self) { label in
                            Button(label) {
                                let cmd = commandMap[label] ?? label
                                tcpClient.send("\(cmd)\n")
                            }
                            .disabled(tcpClient.statusColor != .green)
                            .opacity(tcpClient.statusColor == .green ? 1.0 : 0.5)
                            .padding()
                            .frame(minWidth: 80)
                        }
                    }
                }

                GroupBox(label: Text("Poses")) {
                    HStack(spacing: 10) {
                        ForEach(["Femur 1", "Femur 2", "Femur 3", "Tibia 1", "Tibia 2"], id: \.self) { label in
                            Button(label) {
                                let cmd = commandMap[label] ?? label
                                tcpClient.send("\(cmd)\n")
                            }
                            .disabled(tcpClient.statusColor != .green)
                            .opacity(tcpClient.statusColor == .green ? 1.0 : 0.5)
                            .padding()
                            .frame(minWidth: 100)
                        }
                    }
                }

                // Image view when image is received
                if let imageData = tcpClient.receivedImage {
                    ImageView(
                        imageData: imageData,
                        onAnnotationsComplete: { annotations in
                            appModel.currentAnnotations = annotations
                            appModel.annotationState = .sendingAnnotations
                            tcpClient.sendAnnotations(annotations)
                            appModel.annotationState = .complete
                        }
                    )
                    .onAppear {
                        appModel.currentImageData = imageData
                        appModel.annotationState = .imageReceived
                    }
                }
                
                // Status messages
                VStack(spacing: 5) {
                    Text(tcpClient.statusMessage)
                        .font(.headline)
                        .foregroundColor(tcpClient.statusColor)
                    
                    if !tcpClient.imageTransmissionStatus.isEmpty {
                        Text(tcpClient.imageTransmissionStatus)
                            .font(.subheadline)
                            .foregroundColor(.blue)
                    }
                    
                    // Annotation state indicator
                    Text("State: \(appModel.annotationState)")
                        .font(.caption)
                        .foregroundColor(.gray)
                }
                .padding(.top, 10)
            }
            .padding()
            
            // E-Stop Button in top-right corner
            VStack {
                HStack {
                    Spacer()
                    EStopButton(
                        estopState: $estopState,
                        estopTimer: $estopTimer,
                        isConnected: tcpClient.statusColor == .green,
                        onEStop: {
                            tcpClient.send("KILLALL\n")
                            tcpClient.disconnect()
                            tcpClient.statusMessage = "EMERGENCY STOP ACTIVATED - All systems halted"
                            tcpClient.statusColor = .red
                        }
                    )
                    .padding(.top, 20)
                    .padding(.trailing, 20)
                }
                Spacer()
            }
        }
    }
}

struct EStopButton: View {
    @Binding var estopState: ContentView.EStopState
    @Binding var estopTimer: Timer?
    let isConnected: Bool
    let onEStop: () -> Void
    
    var body: some View {
        Button(action: handleEStopPress) {
            Text(estopState == .normal ? "ESTOP" : "CONFIRM ESTOP")
                .font(.system(size: 16, weight: .bold))
                .foregroundColor(.white)
                .padding(.horizontal, 20)
                .padding(.vertical, 12)
                .background(estopState == .normal ? Color.red : Color.orange)
                .cornerRadius(8)
                .overlay(
                    RoundedRectangle(cornerRadius: 8)
                        .stroke(Color.white, lineWidth: 2)
                )
        }
        .disabled(!isConnected)
        .opacity(isConnected ? 1.0 : 0.5)
        .scaleEffect(estopState == .confirm ? 1.1 : 1.0)
        .animation(.easeInOut(duration: 0.2), value: estopState)
    }
    
    private func handleEStopPress() {
        switch estopState {
        case .normal:
            // First press - change to confirm state
            estopState = .confirm
            startTimer()
        case .confirm:
            // Second press - execute e-stop
            cancelTimer()
            onEStop()
            estopState = .normal
        }
    }
    
    private func startTimer() {
        cancelTimer() // Cancel any existing timer
        estopTimer = Timer.scheduledTimer(withTimeInterval: 5.0, repeats: false) { _ in
            DispatchQueue.main.async {
                estopState = .normal
            }
        }
    }
    
    private func cancelTimer() {
        estopTimer?.invalidate()
        estopTimer = nil
    }
}
