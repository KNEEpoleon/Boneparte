//
//  ContentView.swift
//  BONEparte Surgical Suite
//
//  Main Control Panel - Redesigned with modern surgical aesthetic
//

import SwiftUI
import Network
#if canImport(UIKit)
import UIKit
#endif

// MARK: - Minimal Dark Background
struct GlassmorphismBackground: View {
    var body: some View {
        ZStack {
            // Deep charcoal base
            Color.backgroundPrimary
            
            // Subtle gradient for depth (black to dark gray)
            LinearGradient(
                colors: [
                    Color.black.opacity(0.3),
                    Color(red: 0.15, green: 0.15, blue: 0.16).opacity(0.2)
                ],
                startPoint: .topLeading,
                endPoint: .bottomTrailing
            )
        }
        .ignoresSafeArea()
    }
}

struct ContentView: View {
    @StateObject private var tcpClient = TCPClient(host: "192.168.0.193", port: 5000)
    @State private var selectedDrillSite: String? = nil
    @State private var showEmergencyConfirm = false
    
    // Command mapping
    private let commandMap: [String: String] = [
        "Annotate": "annotate",
        "robot_away": "robot_away",
        "robot_home": "robot_home",
        "Femur 1": "drill_femur_1",
        "Femur 2": "drill_femur_2",
        "Femur 3": "drill_femur_3",
        "Tibia 1": "drill_tibia_1",
        "Tibia 2": "drill_tibia_2"
    ]
    
    // Connection status
    private var isConnected: Bool {
        tcpClient.statusColor == .green
    }
    
    var body: some View {
        GeometryReader { geometry in
            let isCompactLayout = geometry.size.width < 1180
            let padding = EdgeInsets(
                top: Spacing.xl,
                leading: Spacing.xl,
                bottom: Spacing.xl,
                trailing: Spacing.xl
            )
            
            Group {
                if isCompactLayout {
                    VStack(alignment: .leading, spacing: Spacing.xl) {
                        leftColumn
                        Divider()
                            .background(Color.surfaceLight.opacity(0.35))
                        rightColumn
                        Spacer()
                    }
                    .padding(padding)
                } else {
                    HStack(alignment: .top, spacing: Spacing.xl) {
                        leftColumn
                            .frame(maxWidth: 480, alignment: .topLeading)
                        rightColumn
                            .frame(maxWidth: .infinity, alignment: .topLeading)
                    }
                    .padding(padding)
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
                }
            }
        }
        .background(GlassmorphismBackground())
        .alert("Emergency Stop", isPresented: $showEmergencyConfirm) {
            Button("Cancel", role: .cancel) { }
            Button("STOP ALL", role: .destructive) {
                sendCommand("KILLALL")
            }
        } message: {
            Text("This will immediately stop all robotic operations. Continue?")
        }
        .sheet(item: Binding(
            get: { tcpClient.receivedImage.map { ImageData(data: $0) } },
            set: { _ in tcpClient.receivedImage = nil }
        )) { imageData in
            ImageView(imageData: imageData.data) { annotations in
                tcpClient.sendAnnotations(annotations)
            }
        }
    }
    
    private var leftColumn: some View {
        VStack(alignment: .leading, spacing: Spacing.lg) {
            headerSection
            connectionStatusCard
            workflowControlPanel
            Spacer()
        }
    }
    
    private var rightColumn: some View {
        VStack(alignment: .leading, spacing: Spacing.lg) {
            emergencySection
            drillSiteSelection
            Spacer()
        }
    }
    
    // MARK: - Header Section
    
    private var headerSection: some View {
        HStack {
            VStack(alignment: .leading, spacing: Spacing.xs) {
                Text("BONEparte")
                    .font(.displayLarge)
                    .foregroundColor(Color(red: 1.0, green: 0.60, blue: 0.20))
                
                Text("Surgical Control Suite")
                    .font(.bodyMedium)
                    .foregroundColor(.textSecondary)
            }
            
            Spacer()
            
            // Status Indicator
            HStack(spacing: Spacing.sm) {
                Circle()
                    .fill(isConnected ? Color.statusActive : Color.statusDanger)
                    .frame(width: 12, height: 12)
                    .shadow(color: isConnected ? Color.statusActive : Color.statusDanger,
                           radius: 4)
                
                Text(isConnected ? "ONLINE" : "OFFLINE")
                    .font(.labelMedium)
                    .foregroundColor(isConnected ? .statusActive : .statusDanger)
            }
            .padding(.horizontal, Spacing.md)
            .padding(.vertical, Spacing.sm)
            .background(
                Capsule()
                    .fill(Color.surfaceElevated)
            )
        }
    }
    
    // MARK: - Connection Status Card
    
    private var connectionStatusCard: some View {
        StatusCard(
            title: "Network Connection",
            status: isConnected ? .active : .danger,
            statusText: isConnected ? "Connected" : "Disconnected"
        ) {
            VStack(spacing: Spacing.md) {
                // Server Info
                DataRow(
                    label: "ROS Server",
                    value: "192.168.0.193:5000",
                    isMonospaced: true
                )
                
                DataRow(
                    label: "Status",
                    value: tcpClient.statusMessage,
                    valueColor: tcpClient.statusColor
                )
                
                Divider()
                
                // Connection Buttons
                HStack(spacing: Spacing.md) {
                    if isConnected {
                        Button {
                            tcpClient.disconnect()
                            tcpClient.statusMessage = "Disconnected manually"
                            tcpClient.statusColor = .red
                        } label: {
                            HStack {
                                Image(systemName: "wifi.slash")
                                Text("Disconnect")
                            }
                        }
                        .secondaryButton(fullWidth: true)
                    } else {
                        Button {
                            tcpClient.connect()
                        } label: {
                            HStack {
                                Image(systemName: "wifi")
                                Text("Connect to ROS")
                            }
                        }
                        .primaryButton(fullWidth: true)
                    }
                }
            }
        }
    }
    
    // MARK: - Workflow Control Panel
    
    private var workflowControlPanel: some View {
        VStack(spacing: Spacing.md) {
            SectionHeader(
                "Control Panel",
                subtitle: "Core surgical workflow",
                icon: "slider.horizontal.3"
            )
            
            StandardCard(title: "Launch segmentation interface") {
                Button {
                    sendCommand("Annotate")
                } label: {
                    Label("Annotate", systemImage: "pencil.tip.crop.circle")
                }
                .primaryButton(fullWidth: true)
                .disabled(!isConnected)
            }
            
            StandardCard(title: "Send the robot away") {
                Button {
                    sendCommand("robot_away")
                } label: {
                    Label("Send", systemImage: "paperplane.fill")
                }
                .primaryButton(fullWidth: true)
                .disabled(!isConnected)
            }
            
            StandardCard(title: "Bring the robot home") {
                Button {
                    sendCommand("robot_home")
                } label: {
                    Label("Home", systemImage: "house.fill") 
                }
                .primaryButton(fullWidth: true)
                .disabled(!isConnected)
            }
        }
    }
    
    // MARK: - Drill Site Selection
    
    private var drillSiteSelection: some View {
        VStack(spacing: Spacing.md) {
            SectionHeader(
                "Drill Site Selection",
                subtitle: "Select target drill location",
                icon: "target"
            )
            
            StandardCard(title: "Target Sites") {
                VStack(spacing: Spacing.lg) {
                    // Femur sites
                    VStack(alignment: .leading, spacing: Spacing.sm) {
                        HStack {
                            Image(systemName: "circle.fill")
                                .foregroundColor(.femurColor)
                                .font(.system(size: 10))
                            Text("Femur")
                                .font(.labelLarge)
                                .foregroundColor(.textPrimary)
                        }
                        
                        HStack(spacing: Spacing.sm) {
                            ForEach(1...3, id: \.self) { index in
                                Button {
                                    selectDrillSite("Femur \(index)")
                                } label: {
                                    drillSiteButton(
                                        number: index,
                                        color: .femurColor,
                                        isSelected: selectedDrillSite == "Femur \(index)"
                                    )
                                }
                                .disabled(!isConnected)
                            }
                        }
                    }
                    
                    Divider()
                    
                    // Tibia sites
                    VStack(alignment: .leading, spacing: Spacing.sm) {
                        HStack {
                            Image(systemName: "circle.fill")
                                .foregroundColor(.tibiaColor)
                                .font(.system(size: 10))
                            Text("Tibia")
                                .font(.labelLarge)
                                .foregroundColor(.textPrimary)
                        }
                        
                        HStack(spacing: Spacing.sm) {
                            ForEach(1...2, id: \.self) { index in
                                Button {
                                    selectDrillSite("Tibia \(index)")
                                } label: {
                                    drillSiteButton(
                                        number: index,
                                        color: .tibiaColor,
                                        isSelected: selectedDrillSite == "Tibia \(index)"
                                    )
                                }
                                .disabled(!isConnected)
                            }
                            
                            Spacer()
                        }
                    }
                    
                    // Selected site info
                    if let selected = selectedDrillSite {
                        Divider()
                        
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(.statusActive)
                            Text("Selected: \(selected)")
                                .font(.labelMedium)
                                .foregroundColor(.textPrimary)
                            
                            Spacer()
                            
                            Button("Clear") {
                                selectedDrillSite = nil
                            }
                            .font(.labelSmall)
                            .foregroundColor(.surgicalBlue)
                        }
                    }
                }
            }
        }
    }
    
    // MARK: - Emergency Section
    
    private var emergencySection: some View {
        VStack(spacing: Spacing.md) {
            Button {
                showEmergencyConfirm = true
            } label: {
                HStack(spacing: Spacing.md) {
                    Image(systemName: "hand.raised.fill")
                        .font(.system(size: Spacing.iconLarge))
                    Text("EMERGENCY STOP")
                        .font(.buttonLarge)
                }
            }
            .emergencyButton()
            .disabled(!isConnected)
            
            Text("Immediately halts all robotic motion")
                .font(.caption)
                .foregroundColor(.textSecondary)
        }
    }
    
    // MARK: - Helper Views
    
    private func drillSiteButton(number: Int, color: Color, isSelected: Bool) -> some View {
        VStack(spacing: Spacing.xs) {
            ZStack {
                Circle()
                    .fill(isSelected ? color : Color.surfaceElevated)
                    .frame(width: 50, height: 50)
                
                Circle()
                    .stroke(color, lineWidth: isSelected ? 3 : 2)
                    .frame(width: 50, height: 50)
                
                Text("\(number)")
                    .font(.headlineMedium)
                    .fontWeight(.bold)
                    .foregroundColor(isSelected ? .white : color)
            }
        }
        .scaleEffect(isSelected ? 1.1 : 1.0)
        .animation(.spring(response: 0.3), value: isSelected)
    }
    
    // MARK: - Helper Methods
    
    private func sendCommand(_ label: String) {
        guard let command = commandMap[label] else { return }
        tcpClient.send("\(command)\n")
    }
    
    private func selectDrillSite(_ site: String) {
        selectedDrillSite = site
        sendCommand(site)
    }
}

#Preview {
    ContentView()
}

// Helper struct to make Data Identifiable for sheet presentation
struct ImageData: Identifiable {
    let id = UUID()
    let data: Data
}
