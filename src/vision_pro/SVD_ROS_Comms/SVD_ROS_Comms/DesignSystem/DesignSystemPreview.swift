//
//  DesignSystemPreview.swift
//  BONEparte Surgical Suite
//
//  Design System Preview
//  View all design components in one place
//

import SwiftUI

struct DesignSystemPreview: View {
    @State private var selectedTab = 0
    @State private var isConnected = true
    @State private var selectedDrillSite = 0
    
    var body: some View {
        TabView(selection: $selectedTab) {
            // Colors Tab
            ColorsPreview()
                .tabItem {
                    Label("Colors", systemImage: "paintpalette")
                }
                .tag(0)
            
            // Buttons Tab
            ButtonsPreview()
                .tabItem {
                    Label("Buttons", systemImage: "hand.tap")
                }
                .tag(1)
            
            // Cards Tab
            CardsPreview()
                .tabItem {
                    Label("Cards", systemImage: "rectangle.stack")
                }
                .tag(2)
            
            // Typography Tab
            TypographyPreview()
                .tabItem {
                    Label("Typography", systemImage: "textformat")
                }
                .tag(3)
        }
        .frame(width: 800, height: 900)
    }
}


// MARK: - Colors Preview

struct ColorsPreview: View {
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: Spacing.lg) {
                Text("Color Palette")
                    .font(.displayLarge)
                    .padding(.top, Spacing.lg)
                
                // Primary Colors
                ColorSection(
                    title: "Primary Colors",
                    colors: [
                        ("Surgical Blue", Color.surgicalBlue),
                        ("Medical Green", Color.medicalGreen),
                        ("Surgical Steel", Color.surgicalSteel),
                        ("Sterile White", Color.sterileWhite)
                    ]
                )
                
                // Status Colors
                ColorSection(
                    title: "Status Colors",
                    colors: [
                        ("Active", Color.statusActive),
                        ("Warning", Color.statusWarning),
                        ("Danger", Color.statusDanger),
                        ("Idle", Color.statusIdle),
                        ("Processing", Color.statusProcessing)
                    ]
                )
                
                // Anatomical Colors
                ColorSection(
                    title: "Anatomical Colors",
                    colors: [
                        ("Femur", Color.femurColor),
                        ("Tibia", Color.tibiaColor),
                        ("Implant", Color.implantColor),
                        ("Tissue", Color.tissueColor)
                    ]
                )
            }
            .padding(Spacing.windowMargin)
        }
    }
}

struct ColorSection: View {
    let title: String
    let colors: [(String, Color)]
    
    var body: some View {
        VStack(alignment: .leading, spacing: Spacing.md) {
            Text(title)
                .font(.headlineLarge)
            
            LazyVGrid(columns: [
                GridItem(.flexible()),
                GridItem(.flexible()),
                GridItem(.flexible())
            ], spacing: Spacing.md) {
                ForEach(colors, id: \.0) { name, color in
                    VStack {
                        RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                            .fill(color)
                            .frame(height: 80)
                            .shadowMedium()
                        
                        Text(name)
                            .font(.labelSmall)
                            .foregroundColor(.textSecondary)
                    }
                }
            }
        }
    }
}


// MARK: - Buttons Preview

struct ButtonsPreview: View {
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: Spacing.xxl) {
                Text("Button Styles")
                    .font(.displayLarge)
                    .padding(.top, Spacing.lg)
                
                // Primary Buttons
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Primary Buttons")
                        .font(.headlineLarge)
                    
                    HStack(spacing: Spacing.md) {
                        Button("Small") {}.primaryButton(size: .small)
                        Button("Medium") {}.primaryButton(size: .medium)
                        Button("Large") {}.primaryButton(size: .large)
                    }
                    
                    Button("Full Width") {}.primaryButton(fullWidth: true)
                }
                
                // Success & Danger Buttons
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Action Buttons")
                        .font(.headlineLarge)
                    
                    HStack(spacing: Spacing.md) {
                        Button("Confirm") {}.successButton()
                        Button("Delete") {}.dangerButton()
                        Button("Cancel") {}.secondaryButton()
                    }
                }
                
                // Emergency Stop
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Emergency Controls")
                        .font(.headlineLarge)
                    
                    Button {
                        // Emergency stop action
                    } label: {
                        HStack {
                            Image(systemName: "hand.raised.fill")
                            Text("EMERGENCY STOP")
                        }
                    }
                    .emergencyButton()
                }
                
                // Pill Buttons
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Pill Buttons")
                        .font(.headlineLarge)
                    
                    HStack(spacing: Spacing.sm) {
                        Button("Femur 1") {}.pillButton(isSelected: true)
                        Button("Femur 2") {}.pillButton(isSelected: false)
                        Button("Femur 3") {}.pillButton(isSelected: false)
                    }
                }
                
                // Icon Buttons
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Icon Buttons")
                        .font(.headlineLarge)
                    
                    HStack(spacing: Spacing.md) {
                        Button {
                            // Settings action
                        } label: {
                            Image(systemName: "gearshape.fill")
                        }
                        .iconButton(color: .surgicalBlue)
                        
                        Button {
                            // Help action
                        } label: {
                            Image(systemName: "questionmark.circle.fill")
                        }
                        .iconButton(color: .medicalGreen)
                        
                        Button {
                            // Close action
                        } label: {
                            Image(systemName: "xmark.circle.fill")
                        }
                        .iconButton(color: .statusDanger)
                    }
                }
            }
            .padding(Spacing.windowMargin)
        }
    }
}


// MARK: - Cards Preview

struct CardsPreview: View {
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: Spacing.lg) {
                Text("Card Components")
                    .font(.displayLarge)
                    .padding(.top, Spacing.lg)
                
                // Standard Card
                StandardCard(
                    title: "Patient Information",
                    subtitle: "Basic details"
                ) {
                    VStack(spacing: Spacing.sm) {
                        DataRow(label: "Name", value: "John Doe")
                        DataRow(label: "ID", value: "#12345")
                        DataRow(label: "Procedure", value: "Total Knee Arthroplasty")
                    }
                }
                
                // Status Card
                StatusCard(
                    title: "System Status",
                    status: .active,
                    statusText: "Connected"
                ) {
                    VStack(spacing: Spacing.sm) {
                        DataRow(label: "Robot", value: "Ready")
                        DataRow(label: "Camera", value: "Active")
                        DataRow(label: "Network", value: "25ms", valueColor: .statusActive)
                    }
                }
                
                // Info Cards
                HStack(spacing: Spacing.md) {
                    InfoCard(
                        icon: "heart.fill",
                        title: "Heart Rate",
                        value: "72",
                        unit: "bpm",
                        color: .statusActive
                    )
                    
                    InfoCard(
                        icon: "thermometer",
                        title: "Temperature",
                        value: "36.5",
                        unit: "°C",
                        color: .surgicalBlue
                    )
                }
                
                // Alert Banners
                VStack(spacing: Spacing.md) {
                    AlertBanner(
                        type: .success,
                        message: "ArUco marker detected successfully"
                    )
                    
                    AlertBanner(
                        type: .warning,
                        message: "Network latency is high (>100ms)"
                    )
                    
                    AlertBanner(
                        type: .error,
                        message: "Connection to robot lost"
                    )
                }
            }
            .padding(Spacing.windowMargin)
        }
    }
}


// MARK: - Typography Preview

struct TypographyPreview: View {
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: Spacing.xxl) {
                Text("Typography")
                    .font(.displayLarge)
                    .padding(.top, Spacing.lg)
                
                // Display Fonts
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Display Fonts")
                        .font(.headlineLarge)
                    
                    Text("Display Large")
                        .font(.displayLarge)
                    Text("Display Medium")
                        .font(.displayMedium)
                    Text("Display Small")
                        .font(.displaySmall)
                }
                
                // Headline Fonts
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Headline Fonts")
                        .font(.headlineLarge)
                    
                    Text("Headline Large")
                        .font(.headlineLarge)
                    Text("Headline Medium")
                        .font(.headlineMedium)
                    Text("Headline Small")
                        .font(.headlineSmall)
                }
                
                // Body Fonts
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Body Fonts")
                        .font(.headlineLarge)
                    
                    Text("Body Large - Lorem ipsum dolor sit amet, consectetur adipiscing elit.")
                        .font(.bodyLarge)
                    Text("Body Medium - Lorem ipsum dolor sit amet, consectetur adipiscing elit.")
                        .font(.bodyMedium)
                    Text("Body Small - Lorem ipsum dolor sit amet, consectetur adipiscing elit.")
                        .font(.bodySmall)
                }
                
                // Monospaced Fonts
                VStack(alignment: .leading, spacing: Spacing.md) {
                    Text("Monospaced (Data)")
                        .font(.headlineLarge)
                    
                    Text("Position: (0.123, 0.456, 0.789)")
                        .font(.monoLarge)
                    Text("Orientation: (0.0, 0.0, 0.707, 0.707)")
                        .font(.monoMedium)
                    Text("Timestamp: 2025-11-12 14:30:45")
                        .font(.monoSmall)
                }
            }
            .padding(Spacing.windowMargin)
        }
    }
}


// MARK: - Preview

#Preview {
    DesignSystemPreview()
}

