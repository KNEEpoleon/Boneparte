//
//  ButtonStyles.swift
//  BONEparte Surgical Suite
//
//  Design System: Custom Button Styles
//  Professional, medical-grade button designs
//

import SwiftUI

// MARK: - Primary Action Button

struct PrimaryButtonStyle: ButtonStyle {
    var isFullWidth: Bool = false
    var size: ButtonSize = .medium
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(size.font)
            .foregroundColor(.white)
            .frame(maxWidth: isFullWidth ? .infinity : nil)
            .frame(height: size.height)
            .padding(.horizontal, Spacing.buttonPadding)
            .background(
                RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                    .fill(Color(red: 1.0, green: 0.60, blue: 0.20))
                    .opacity(configuration.isPressed ? 0.8 : 1.0)
            )
            .shadowMedium()
            .scaleEffect(configuration.isPressed ? 0.98 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.7), value: configuration.isPressed)
    }
}


// MARK: - Success Button (Confirm Actions)

struct SuccessButtonStyle: ButtonStyle {
    var isFullWidth: Bool = false
    var size: ButtonSize = .medium
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(size.font)
            .foregroundColor(.white)
            .frame(maxWidth: isFullWidth ? .infinity : nil)
            .frame(height: size.height)
            .padding(.horizontal, Spacing.buttonPadding)
            .background(
                RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                    .fill(Color.buttonSuccess)
                    .opacity(configuration.isPressed ? 0.8 : 1.0)
            )
            .shadowMedium()
            .scaleEffect(configuration.isPressed ? 0.98 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.7), value: configuration.isPressed)
    }
}


// MARK: - Danger Button (Destructive Actions)

struct DangerButtonStyle: ButtonStyle {
    var isFullWidth: Bool = false
    var size: ButtonSize = .medium
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(size.font)
            .foregroundColor(.white)
            .frame(maxWidth: isFullWidth ? .infinity : nil)
            .frame(height: size.height)
            .padding(.horizontal, Spacing.buttonPadding)
            .background(
                RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                    .fill(Color.buttonDanger)
                    .opacity(configuration.isPressed ? 0.8 : 1.0)
            )
            .shadowMedium()
            .scaleEffect(configuration.isPressed ? 0.98 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.7), value: configuration.isPressed)
    }
}


// MARK: - Secondary Button (Less Prominent)

struct SecondaryButtonStyle: ButtonStyle {
    var isFullWidth: Bool = false
    var size: ButtonSize = .medium
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(size.font)
            .foregroundColor(.textPrimary)
            .frame(maxWidth: isFullWidth ? .infinity : nil)
            .frame(height: size.height)
            .padding(.horizontal, Spacing.buttonPadding)
            .background(
                RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                    .fill(Color.surfaceElevated)
                    .opacity(configuration.isPressed ? 0.7 : 1.0)
            )
            .overlay(
                RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                    .stroke(Color(red: 0.40, green: 0.40, blue: 0.42), lineWidth: Spacing.borderMedium)
            )
            .shadowSubtle()
            .scaleEffect(configuration.isPressed ? 0.98 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.7), value: configuration.isPressed)
    }
}


// MARK: - Emergency Stop Button

struct EmergencyStopButtonStyle: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.buttonLarge)
            .fontWeight(.bold)
            .foregroundColor(.white)
            .frame(height: Spacing.buttonLarge + 10)
            .frame(maxWidth: .infinity)
            .background(
                RoundedRectangle(cornerRadius: Spacing.cornerLarge)
                    .fill(
                        LinearGradient(
                            colors: [Color.emergencyRed, Color.emergencyRed.opacity(0.8)],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        )
                    )
                    .opacity(configuration.isPressed ? 0.8 : 1.0)
            )
            .overlay(
                RoundedRectangle(cornerRadius: Spacing.cornerLarge)
                    .stroke(Color.white, lineWidth: 3)
            )
            .shadowLarge()
            .scaleEffect(configuration.isPressed ? 0.95 : 1.0)
            .animation(.spring(response: 0.2, dampingFraction: 0.6), value: configuration.isPressed)
    }
}


// MARK: - Pill Button (Compact, Rounded)

struct PillButtonStyle: ButtonStyle {
    var color: Color = .buttonPrimary
    var isSelected: Bool = false
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.buttonSmall)
            .foregroundColor(isSelected ? .white : color)
            .padding(.horizontal, Spacing.md)
            .padding(.vertical, Spacing.sm)
            .background(
                Capsule()
                    .fill(isSelected ? color : Color.surfaceElevated)
                    .opacity(configuration.isPressed ? 0.7 : 1.0)
            )
            .overlay(
                Capsule()
                    .stroke(color, lineWidth: isSelected ? 0 : Spacing.borderThin)
            )
            .scaleEffect(configuration.isPressed ? 0.95 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.7), value: configuration.isPressed)
    }
}


// MARK: - Icon Button (Circular)

struct IconButtonStyle: ButtonStyle {
    var color: Color = .buttonPrimary
    var size: CGFloat = 44
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.system(size: 20, weight: .semibold))
            .foregroundColor(color)
            .frame(width: size, height: size)
            .background(
                Circle()
                    .fill(Color.surfaceElevated)
                    .opacity(configuration.isPressed ? 0.7 : 1.0)
            )
            .overlay(
                Circle()
                    .stroke(color, lineWidth: Spacing.borderMedium)
            )
            .shadowSubtle()
            .scaleEffect(configuration.isPressed ? 0.9 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.7), value: configuration.isPressed)
    }
}


// MARK: - Card Button (Tappable Card)

struct CardButtonStyle: ButtonStyle {
    var isSelected: Bool = false
    
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .frame(maxWidth: .infinity)
            .cardPadding()
            .background(
                RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                    .fill(isSelected ? Color.surgicalBlue.opacity(0.2) : Color.surfaceElevated)
                    .opacity(configuration.isPressed ? 0.8 : 1.0)
            )
            .overlay(
                RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                    .stroke(isSelected ? Color.surgicalBlue : Color.clear, lineWidth: Spacing.borderMedium)
            )
            .shadowMedium()
            .scaleEffect(configuration.isPressed ? 0.98 : 1.0)
            .animation(.spring(response: 0.3, dampingFraction: 0.7), value: configuration.isPressed)
    }
}


// MARK: - Button Size Enum

enum ButtonSize {
    case small
    case medium
    case large
    
    var height: CGFloat {
        switch self {
        case .small:
            return Spacing.buttonSmall
        case .medium:
            return Spacing.buttonMedium
        case .large:
            return Spacing.buttonLarge
        }
    }
    
    var font: Font {
        switch self {
        case .small:
            return .buttonSmall
        case .medium:
            return .buttonMedium
        case .large:
            return .buttonLarge
        }
    }
}


// MARK: - Convenience Extensions

extension Button {
    
    /// Apply primary button style
    func primaryButton(fullWidth: Bool = false, size: ButtonSize = .medium) -> some View {
        self.buttonStyle(PrimaryButtonStyle(isFullWidth: fullWidth, size: size))
    }
    
    /// Apply success button style
    func successButton(fullWidth: Bool = false, size: ButtonSize = .medium) -> some View {
        self.buttonStyle(SuccessButtonStyle(isFullWidth: fullWidth, size: size))
    }
    
    /// Apply danger button style
    func dangerButton(fullWidth: Bool = false, size: ButtonSize = .medium) -> some View {
        self.buttonStyle(DangerButtonStyle(isFullWidth: fullWidth, size: size))
    }
    
    /// Apply secondary button style
    func secondaryButton(fullWidth: Bool = false, size: ButtonSize = .medium) -> some View {
        self.buttonStyle(SecondaryButtonStyle(isFullWidth: fullWidth, size: size))
    }
    
    /// Apply emergency stop style
    func emergencyButton() -> some View {
        self.buttonStyle(EmergencyStopButtonStyle())
    }
    
    /// Apply pill button style
    func pillButton(color: Color = .buttonPrimary, isSelected: Bool = false) -> some View {
        self.buttonStyle(PillButtonStyle(color: color, isSelected: isSelected))
    }
    
    /// Apply icon button style
    func iconButton(color: Color = .buttonPrimary, size: CGFloat = 44) -> some View {
        self.buttonStyle(IconButtonStyle(color: color, size: size))
    }
    
    /// Apply card button style
    func cardButton(isSelected: Bool = false) -> some View {
        self.buttonStyle(CardButtonStyle(isSelected: isSelected))
    }
}

