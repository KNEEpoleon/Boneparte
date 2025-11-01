//
//  ContentView.swift
//  ArucoTransform
//
//  Main UI window with connection controls and transform enable button
//

import SwiftUI
import RealityKit
import RealityKitContent

struct ContentView: View {
    @Environment(AppModel.self) private var appModel
    @Environment(\.openImmersiveSpace) var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) var dismissImmersiveSpace
    
    @State private var tcpClient: TCPClient?
    @State private var serverIPInput = "192.168.1.100"
    
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
                
                HStack {
                    Text("IP Address:")
                    TextField("192.168.1.100", text: $serverIPInput)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 200)
                }
                
                HStack {
                    Text("Port:")
                    Text("5001")
                        .foregroundColor(.secondary)
                }
                
                Button(action: {
                    if appModel.isConnected {
                        disconnectFromServer()
                    } else {
                        connectToServer()
                    }
                }) {
                    HStack {
                        Image(systemName: appModel.isConnected ? "wifi.circle.fill" : "wifi.slash")
                        Text(appModel.isConnected ? "Disconnect" : "Connect")
                    }
                    .frame(maxWidth: .infinity)
                }
                .buttonStyle(.borderedProminent)
                .tint(appModel.isConnected ? .red : .blue)
                
                Text(appModel.connectionStatus)
                    .font(.caption)
                    .foregroundColor(appModel.isConnected ? .green : .secondary)
            }
            .padding()
            .background(Color.secondary.opacity(0.1))
            .cornerRadius(10)
            
            Divider()
            
            // Transform section
            VStack(alignment: .leading, spacing: 10) {
                Text("Drill Site Visualization")
                    .font(.headline)
                
                if !appModel.isConnected {
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
                .disabled(!appModel.isConnected)
                
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
                if appModel.isConnected && appModel.isTransformEnabled {
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
            serverIPInput = appModel.serverIP
        }
    }
    
    private func connectToServer() {
        appModel.serverIP = serverIPInput
        tcpClient = TCPClient(host: appModel.serverIP, port: appModel.serverPort)
        tcpClient?.onDrillPosesReceived = { drillSites in
            appModel.drillSites = drillSites
        }
        tcpClient?.onConnectionStateChanged = { isConnected, status in
            appModel.isConnected = isConnected
            appModel.connectionStatus = status
        }
        tcpClient?.connect()
    }
    
    private func disconnectFromServer() {
        tcpClient?.disconnect()
        tcpClient = nil
        appModel.isConnected = false
        appModel.connectionStatus = "Disconnected"
        appModel.drillSites = []
    }
}

#Preview(windowStyle: .automatic) {
    ContentView()
        .environment(AppModel())
}

