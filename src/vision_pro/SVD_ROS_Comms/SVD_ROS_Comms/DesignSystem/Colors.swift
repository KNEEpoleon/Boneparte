//
//  Colors.swift
//  BONEparte Surgical Suite
//
//  Design System: Color Palette
//  Surgical-grade professional color scheme
//

import SwiftUI

extension Color {
    
    // MARK: - Primary Colors (Minimal Black/Gray/Orange)
    
    /// Primary action - vibrant orange
    static let surgicalBlue = Color(red: 1.0, green: 0.60, blue: 0.20)
    
    /// Success - orange (unified palette)
    static let medicalGreen = Color(red: 1.0, green: 0.60, blue: 0.20)
    
    /// Pure white
    static let sterileWhite = Color.white
    
    /// Dark gray - high contrast
    static let surgicalSteel = Color(red: 0.20, green: 0.20, blue: 0.22)
    
    
    // MARK: - Status Colors
    
    /// Active/Connected state - bright green
    static let statusActive = Color(red: 0.20, green: 0.85, blue: 0.40)
    
    /// Warning state - amber/orange
    static let statusWarning = Color(red: 1.00, green: 0.60, blue: 0.00)
    
    /// Danger/Error state - red
    static let statusDanger = Color(red: 0.90, green: 0.20, blue: 0.20)
    
    /// Idle/Inactive state - gray
    static let statusIdle = Color(red: 0.55, green: 0.55, blue: 0.57)
    
    /// Processing/In-progress - blue
    static let statusProcessing = Color(red: 0.20, green: 0.60, blue: 0.90)
    
    
    // MARK: - UI Surface Colors (Minimal Dark)
    
    /// Primary background - deep charcoal
    static let backgroundPrimary = Color(red: 0.10, green: 0.10, blue: 0.11)
    
    /// Secondary background - slightly lighter
    static let backgroundSecondary = Color(red: 0.13, green: 0.13, blue: 0.14)
    
    /// Card/Panel surface - dark gray elevated
    static let surfaceElevated = Color(red: 0.16, green: 0.16, blue: 0.17)
    
    /// Card/Panel surface - lighter alternative
    static let surfaceLight = Color(red: 0.18, green: 0.18, blue: 0.19)
    
    
    // MARK: - Text Colors (High Contrast for Bright Light)
    
    /// Primary text - pure white
    static let textPrimary = Color.white
    
    /// Secondary text - light gray
    static let textSecondary = Color(red: 0.70, green: 0.70, blue: 0.72)
    
    /// Tertiary text - medium gray
    static let textTertiary = Color(red: 0.50, green: 0.50, blue: 0.52)
    
    /// Inverse text - for light backgrounds
    static let textInverse = Color.black
    
    
    // MARK: - Anatomical Colors (Orange Accent)
    
    /// Femur - bright orange
    static let femurColor = Color(red: 1.0, green: 0.60, blue: 0.20)
    
    /// Tibia - muted orange
    static let tibiaColor = Color(red: 1.0, green: 0.70, blue: 0.40)
    
    /// Implant/metal - metallic
    static let implantColor = Color(red: 0.75, green: 0.78, blue: 0.82)
    
    /// Soft tissue - pink/red
    static let tissueColor = Color(red: 0.90, green: 0.65, blue: 0.70)
    
    
    // MARK: - Interactive Elements
    
    /// Button primary - main actions
    static let buttonPrimary = surgicalBlue
    
    /// Button success - confirmations
    static let buttonSuccess = medicalGreen
    
    /// Button danger - destructive actions
    static let buttonDanger = statusDanger
    
    /// Button secondary - less important actions
    static let buttonSecondary = surgicalSteel
    
    
    // MARK: - Semantic Colors
    
    /// Emergency stop - bright red
    static let emergencyRed = Color(red: 1.00, green: 0.15, blue: 0.15)
    
    /// Network connection - green
    static let networkConnected = statusActive
    
    /// Network disconnected - red
    static let networkDisconnected = statusDanger
    
    /// AR detected - cyan
    static let arDetected = Color(red: 0.20, green: 0.80, blue: 0.90)
    
    /// AR not detected - gray
    static let arNotDetected = statusIdle
    
    
    // MARK: - Gradient Presets
    
    /// Header gradient - surgical blue fade
    static let headerGradient = LinearGradient(
        colors: [surgicalBlue, surgicalBlue.opacity(0.7)],
        startPoint: .topLeading,
        endPoint: .bottomTrailing
    )
    
    /// Success gradient - green fade
    static let successGradient = LinearGradient(
        colors: [medicalGreen, medicalGreen.opacity(0.7)],
        startPoint: .topLeading,
        endPoint: .bottomTrailing
    )
    
    /// Danger gradient - red fade
    static let dangerGradient = LinearGradient(
        colors: [statusDanger, statusDanger.opacity(0.7)],
        startPoint: .topLeading,
        endPoint: .bottomTrailing
    )
    
    
    // MARK: - Helper Methods
    
    /// Returns appropriate text color for given background
    static func textColor(for background: Color) -> Color {
        // Simplified - in production, calculate luminance
        return textPrimary
    }
    
    /// Returns status color based on connection state
    static func connectionColor(isConnected: Bool) -> Color {
        return isConnected ? statusActive : statusDanger
    }
}


// MARK: - Color Scheme Support

extension ColorScheme {
    /// Returns appropriate surface color for current scheme
    func surfaceColor() -> Color {
        switch self {
        case .dark:
            return Color.surfaceElevated
        case .light:
            return Color.surfaceLight
        @unknown default:
            return Color.surfaceElevated
        }
    }
}

