//
//  ContentView.swift
//  ArucoTransform
//
//  Main UI window with connection controls and transform enable button
//

import SwiftUI
import RealityKit
// import RealityKitContent

struct ContentView: View {
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    
    // ISSUE #3: Use @StateObject with immediate initialization (like SVD_ROS_Comms)
    // ISSUE #2: Hardcode IP directly in initialization (no appModel override)
    @StateObject private var tcpClient = ArucoTCPClient(host: "192.168.0.193", port: 5000)
    
    var body: some View {
        VStack(spacing: 20) {
            Text("ArUco Transform")
                .font(.title)
                .fontWeight(.bold)
            
            Divider()
            
            // Connection section
            VStack(alignment: .leading, spacing: 10) {
                Text("ROS Server Connection")
                    .font(.headline)
                
                // ISSUE #4: Display static IP/Port (removed TextField)
                HStack {
                    Text("Server:")
                    Text("192.168.0.193:5000")
                        .foregroundColor(.secondary)
                        .fontWeight(.medium)
                }
                
                Button(action: {
                    if tcpClient.isConnected {
                        tcpClient.disconnect()
                        appModel.isConnected = false
                        appModel.drillSites = []
                    } else {
                        tcpClient.connect()
                    }
                }) {
                    HStack {
                        Image(systemName: tcpClient.isConnected ? "wifi.circle.fill" : "wifi.slash")
                        Text(tcpClient.isConnected ? "Disconnect" : "Connect")
                    }
                    .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(tcpClient.isConnected ? .red : .blue)
                
                Text(tcpClient.statusMessage)
                    .font(.caption)
                    .foregroundColor(tcpClient.statusColor)
            }
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(10)
            
            Divider()
            
            // Transform section
            VStack(alignment: .leading, spacing: 10) {
                Text("Drill Site Visualization")
                    .font(.headline)
                
                if !tcpClient.isConnected {
                    Text("Please connect to ROS server first")
                        .foregroundColor(.secondary)
                        .font(.caption)
                }
                
                Button(action: {
                    Task {
                        if appModel.immersiveSpaceState == .open {
                            await dismissImmersiveSpace()
                            appModel.isTransformEnabled = false
                        } else {
                            await openImmersiveSpace(id: appModel.immersiveSpaceID)
                            appModel.isTransformEnabled = true
                        }
                    }
                }) {
                    HStack {
                        Image(systemName: appModel.isTransformEnabled ? "eye.fill" : "eye.slash")
                        Text(appModel.isTransformEnabled ? "Hide Drill Sites" : "Show Drill Sites")
                    }
                    .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(appModel.isTransformEnabled ? .orange : .green)
                .disabled(!tcpClient.isConnected)
                
                if appModel.immersiveSpaceState == .open {
                    if appModel.arucoTransformEstablished {
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(.green)
                            Text("ArUco marker detected & localized")
                                .font(.caption)
                        }
                        
                        Text("Drill sites: \(appModel.drillSites.count)")
                            .font(.caption)
                            .foregroundColor(.secondary)
                    } else {
                        HStack {
                            ProgressView()
                                .scaleEffect(0.7)
                            Text("Looking for ArUco marker (ID=0)...")
                                .font(.caption)
                        }
                        
                        Text("Point your Vision Pro at the 15cm ArUco marker")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                }
            }
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(10)
            
            Spacer()
            
            // Status footer
            VStack(spacing: 5) {
                if tcpClient.isConnected && appModel.isTransformEnabled {
                    Text("System Active")
                        .font(.caption)
                        .foregroundColor(.green)
                } else {
                    Text("System Idle")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
        }
        .padding(30)
        .frame(width: 400, height: 550)
        .onAppear {
            // Setup drill pose callback
            tcpClient.onDrillPosesReceived = { drillSites in
                appModel.drillSites = drillSites
                appModel.isConnected = true
            }
        }
    }
}

#Preview(windowStyle: .automatic) {
    ContentView()
        .environment(AppModel())
}

