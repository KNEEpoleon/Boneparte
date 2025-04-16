import SwiftUI
import Network

struct ContentView: View {
    @StateObject private var tcpClient = TCPClient(host: "192.168.0.193", port: 5000)

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

            Text(tcpClient.statusMessage)
                .font(.headline)
                .foregroundColor(tcpClient.statusColor)
                .padding(.top, 10)
        }
        .padding()
    }
}
