//
//  CardStyles.swift
//  BONEparte Surgical Suite
//
//  Design System: Card & Panel Components
//  Reusable container styles for content organization
//

import SwiftUI

// MARK: - Standard Card

struct StandardCard<Content: View>: View {
    let title: String?
    let subtitle: String?
    let content: Content
    
    init(
        title: String? = nil,
        subtitle: String? = nil,
        @ViewBuilder content: () -> Content
    ) {
        self.title = title
        self.subtitle = subtitle
        self.content = content()
    }
    
    var body: some View {
        VStack(alignment: .leading, spacing: Spacing.md) {
            // Header
            if let title = title {
                VStack(alignment: .leading, spacing: Spacing.xs) {
                    Text(title)
                        .font(.headlineMedium)
                        .foregroundColor(.textPrimary)
                    
                    if let subtitle = subtitle {
                        Text(subtitle)
                            .font(.caption)
                            .foregroundColor(.textSecondary)
                    }
                }
            }
            
            // Content
            content
        }
        .cardPadding()
        .background(
            RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                .fill(Color.surfaceElevated)
        )
        .shadowMedium()
    }
}


// MARK: - Status Card (with indicator)

struct StatusCard<Content: View>: View {
    let title: String
    let status: StatusType
    let statusText: String
    let content: Content
    
    init(
        title: String,
        status: StatusType,
        statusText: String,
        @ViewBuilder content: () -> Content
    ) {
        self.title = title
        self.status = status
        self.statusText = statusText
        self.content = content()
    }
    
    var body: some View {
        VStack(alignment: .leading, spacing: Spacing.md) {
            // Header with status
            HStack {
                Text(title)
                    .font(.headlineMedium)
                    .foregroundColor(.textPrimary)
                
                Spacer()
                
                HStack(spacing: Spacing.xs) {
                    Circle()
                        .fill(status.color)
                        .frame(width: 10, height: 10)
                    
                    Text(statusText)
                        .font(.labelSmall)
                        .foregroundColor(status.color)
                }
            }
            
            Divider()
            
            // Content
            content
        }
        .cardPadding()
        .background(
            RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                .fill(Color.surfaceElevated)
        )
        .overlay(
            RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                .stroke(status.color.opacity(0.3), lineWidth: Spacing.borderThin)
        )
        .shadowMedium()
    }
}


// MARK: - Info Card (Simple Information Display)

struct InfoCard: View {
    let icon: String
    let title: String
    let value: String
    let unit: String?
    let color: Color
    
    init(
        icon: String,
        title: String,
        value: String,
        unit: String? = nil,
        color: Color = .surgicalBlue
    ) {
        self.icon = icon
        self.title = title
        self.value = value
        self.unit = unit
        self.color = color
    }
    
    var body: some View {
        HStack(spacing: Spacing.md) {
            // Icon
            Image(systemName: icon)
                .font(.system(size: Spacing.iconLarge))
                .foregroundColor(color)
                .frame(width: 50, height: 50)
                .background(
                    Circle()
                        .fill(color.opacity(0.1))
                )
            
            VStack(alignment: .leading, spacing: Spacing.xs) {
                Text(title)
                    .font(.labelMedium)
                    .foregroundColor(.textSecondary)
                
                HStack(alignment: .firstTextBaseline, spacing: Spacing.xs) {
                    Text(value)
                        .font(.numericLarge)
                        .foregroundColor(.textPrimary)
                    
                    if let unit = unit {
                        Text(unit)
                            .font(.labelMedium)
                            .foregroundColor(.textSecondary)
                    }
                }
            }
            
            Spacer()
        }
        .cardPadding()
        .background(
            RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                .fill(Color.surfaceElevated)
        )
        .shadowSubtle()
    }
}


// MARK: - Section Header

struct SectionHeader: View {
    let title: String
    let subtitle: String?
    let icon: String?
    let action: (() -> Void)?
    let actionLabel: String?
    
    init(
        _ title: String,
        subtitle: String? = nil,
        icon: String? = nil,
        actionLabel: String? = nil,
        action: (() -> Void)? = nil
    ) {
        self.title = title
        self.subtitle = subtitle
        self.icon = icon
        self.actionLabel = actionLabel
        self.action = action
    }
    
    var body: some View {
        HStack(alignment: .center, spacing: Spacing.md) {
            // Icon (optional)
            if let icon = icon {
                Image(systemName: icon)
                    .font(.system(size: Spacing.iconMedium))
                    .foregroundColor(.surgicalBlue)
            }
            
            // Title & Subtitle
            VStack(alignment: .leading, spacing: Spacing.xs) {
                Text(title)
                    .font(.headlineLarge)
                    .foregroundColor(.textPrimary)
                
                if let subtitle = subtitle {
                    Text(subtitle)
                        .font(.caption)
                        .foregroundColor(.textSecondary)
                }
            }
            
            Spacer()
            
            // Action button (optional)
            if let action = action, let actionLabel = actionLabel {
                Button(action: action) {
                    Text(actionLabel)
                        .font(.labelMedium)
                        .foregroundColor(.surgicalBlue)
                }
            }
        }
        .padding(.vertical, Spacing.sm)
    }
}


// MARK: - Group Box Style

struct SurgicalGroupBoxStyle: GroupBoxStyle {
    func makeBody(configuration: Configuration) -> some View {
        VStack(alignment: .leading, spacing: Spacing.md) {
            configuration.label
                .font(.headlineSmall)
                .foregroundColor(.textPrimary)
            
            Divider()
            
            configuration.content
        }
        .cardPadding()
        .background(
            RoundedRectangle(cornerRadius: Spacing.cornerMedium)
                .fill(Color.surfaceElevated)
        )
        .shadowMedium()
    }
}


// MARK: - Data Row (Label-Value Pair)

struct DataRow: View {
    let label: String
    let value: String
    let valueColor: Color
    let isMonospaced: Bool
    
    init(
        label: String,
        value: String,
        valueColor: Color = .textPrimary,
        isMonospaced: Bool = false
    ) {
        self.label = label
        self.value = value
        self.valueColor = valueColor
        self.isMonospaced = isMonospaced
    }
    
    var body: some View {
        HStack {
            Text(label)
                .font(.labelMedium)
                .foregroundColor(.textSecondary)
            
            Spacer()
            
            Text(value)
                .font(isMonospaced ? .monoMedium : .bodyMedium)
                .foregroundColor(valueColor)
        }
        .padding(.vertical, Spacing.xs)
    }
}


// MARK: - Alert Banner

struct AlertBanner: View {
    let type: AlertType
    let message: String
    let dismissAction: (() -> Void)?
    
    init(
        type: AlertType,
        message: String,
        dismissAction: (() -> Void)? = nil
    ) {
        self.type = type
        self.message = message
        self.dismissAction = dismissAction
    }
    
    var body: some View {
        HStack(spacing: Spacing.md) {
            // Icon
            Image(systemName: type.icon)
                .font(.system(size: Spacing.iconMedium))
                .foregroundColor(type.color)
            
            // Message
            Text(message)
                .font(.bodyMedium)
                .foregroundColor(.textPrimary)
            
            Spacer()
            
            // Dismiss button
            if let dismissAction = dismissAction {
                Button(action: dismissAction) {
                    Image(systemName: "xmark.circle.fill")
                        .font(.system(size: Spacing.iconMedium))
                        .foregroundColor(.textSecondary)
                }
            }
        }
        .padding(Spacing.md)
        .background(
            RoundedRectangle(cornerRadius: Spacing.cornerSmall)
                .fill(type.color.opacity(0.15))
        )
        .overlay(
            RoundedRectangle(cornerRadius: Spacing.cornerSmall)
                .stroke(type.color, lineWidth: Spacing.borderThin)
        )
    }
}

enum AlertType {
    case info
    case success
    case warning
    case error
    
    var icon: String {
        switch self {
        case .info:
            return "info.circle.fill"
        case .success:
            return "checkmark.circle.fill"
        case .warning:
            return "exclamationmark.triangle.fill"
        case .error:
            return "xmark.octagon.fill"
        }
    }
    
    var color: Color {
        switch self {
        case .info:
            return .surgicalBlue
        case .success:
            return .statusActive
        case .warning:
            return .statusWarning
        case .error:
            return .statusDanger
        }
    }
}


// MARK: - Convenience Extensions

extension View {
    
    /// Wrap content in a standard card
    func asCard(title: String? = nil, subtitle: String? = nil) -> some View {
        StandardCard(title: title, subtitle: subtitle) {
            self
        }
    }
    
    /// Wrap content in a status card
    func asStatusCard(
        title: String,
        status: StatusType,
        statusText: String
    ) -> some View {
        StatusCard(title: title, status: status, statusText: statusText) {
            self
        }
    }
}

