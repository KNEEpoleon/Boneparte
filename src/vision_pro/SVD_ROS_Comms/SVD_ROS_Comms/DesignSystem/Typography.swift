//
//  Typography.swift
//  BONEparte Surgical Suite
//
//  Design System: Typography
//  Professional, medical-grade text styling
//

import SwiftUI

extension Font {
    
    // MARK: - Display Fonts (Headers & Titles)
    
    /// Extra large display - app title, main headers
    static let displayLarge = Font.system(size: 34, weight: .bold, design: .default)
    
    /// Large display - section headers
    static let displayMedium = Font.system(size: 28, weight: .semibold, design: .default)
    
    /// Small display - subsection headers
    static let displaySmall = Font.system(size: 24, weight: .medium, design: .default)
    
    
    // MARK: - Headline Fonts
    
    /// Primary headline - card titles
    static let headlineLarge = Font.system(size: 20, weight: .semibold, design: .default)
    
    /// Secondary headline - subheaders
    static let headlineMedium = Font.system(size: 18, weight: .medium, design: .default)
    
    /// Small headline - labels
    static let headlineSmall = Font.system(size: 16, weight: .medium, design: .default)
    
    
    // MARK: - Body Fonts
    
    /// Large body - primary content
    static let bodyLarge = Font.system(size: 17, weight: .regular, design: .default)
    
    /// Medium body - standard text
    static let bodyMedium = Font.system(size: 15, weight: .regular, design: .default)
    
    /// Small body - secondary text
    static let bodySmall = Font.system(size: 13, weight: .regular, design: .default)
    
    
    // MARK: - Label Fonts
    
    /// Large label - button text, form labels
    static let labelLarge = Font.system(size: 16, weight: .semibold, design: .default)
    
    /// Medium label - standard labels
    static let labelMedium = Font.system(size: 14, weight: .medium, design: .default)
    
    /// Small label - metadata, timestamps
    static let labelSmall = Font.system(size: 12, weight: .regular, design: .default)
    
    
    // MARK: - Caption Fonts
    
    /// Caption - help text, footnotes
    static let caption = Font.system(size: 12, weight: .regular, design: .default)
    
    /// Small caption - fine print
    static let captionSmall = Font.system(size: 10, weight: .regular, design: .default)
    
    
    // MARK: - Monospaced Fonts (for data, coordinates)
    
    /// Monospaced large - coordinates, values
    static let monoLarge = Font.system(size: 16, weight: .regular, design: .monospaced)
    
    /// Monospaced medium - data display
    static let monoMedium = Font.system(size: 14, weight: .regular, design: .monospaced)
    
    /// Monospaced small - compact data
    static let monoSmall = Font.system(size: 12, weight: .regular, design: .monospaced)
    
    
    // MARK: - Button Fonts
    
    /// Large button - primary actions
    static let buttonLarge = Font.system(size: 18, weight: .semibold, design: .default)
    
    /// Medium button - standard buttons
    static let buttonMedium = Font.system(size: 16, weight: .semibold, design: .default)
    
    /// Small button - compact buttons
    static let buttonSmall = Font.system(size: 14, weight: .medium, design: .default)
    
    
    // MARK: - Special Purpose Fonts
    
    /// Status text - system messages
    static let statusText = Font.system(size: 15, weight: .medium, design: .default)
    
    /// Timer/Counter - elapsed time, counts
    static let timerFont = Font.system(size: 20, weight: .medium, design: .monospaced)
    
    /// Numeric display - large numbers
    static let numericLarge = Font.system(size: 28, weight: .bold, design: .monospaced)
}


// MARK: - Text Modifiers

extension Text {
    
    /// Apply display large style
    func displayLarge() -> Text {
        self.font(.displayLarge)
    }
    
    /// Apply headline style with color
    func headline(color: Color = .textPrimary) -> Text {
        self.font(.headlineMedium)
            .foregroundColor(color)
    }
    
    /// Apply body style with color
    func body(color: Color = .textPrimary) -> Text {
        self.font(.bodyMedium)
            .foregroundColor(color)
    }
    
    /// Apply caption style with color
    func caption(color: Color = .textSecondary) -> Text {
        self.font(.caption)
            .foregroundColor(color)
    }
    
    /// Apply monospaced style for data display
    func dataValue() -> Text {
        self.font(.monoMedium)
            .foregroundColor(.textPrimary)
    }
    
    /// Apply status text style with appropriate color
    func statusText(status: StatusType) -> Text {
        self.font(.statusText)
            .foregroundColor(status.color)
    }
}


// MARK: - Status Type

enum StatusType {
    case active
    case warning
    case danger
    case idle
    case processing
    
    var color: Color {
        switch self {
        case .active:
            return .statusActive
        case .warning:
            return .statusWarning
        case .danger:
            return .statusDanger
        case .idle:
            return .statusIdle
        case .processing:
            return .statusProcessing
        }
    }
}


// MARK: - Line Height & Spacing

struct TextSpacing {
    /// Tight line spacing - compact layouts
    static let tight: CGFloat = 1.1
    
    /// Normal line spacing - standard readability
    static let normal: CGFloat = 1.4
    
    /// Loose line spacing - enhanced readability
    static let loose: CGFloat = 1.6
}


// MARK: - View Modifier for Consistent Typography

struct TypographyModifier: ViewModifier {
    let style: TypographyStyle
    
    func body(content: Content) -> some View {
        content
            .font(style.font)
            .foregroundColor(style.color)
            .lineSpacing(style.lineSpacing)
    }
}

struct TypographyStyle {
    let font: Font
    let color: Color
    let lineSpacing: CGFloat
    
    static let titlePrimary = TypographyStyle(
        font: .displayMedium,
        color: .textPrimary,
        lineSpacing: TextSpacing.normal
    )
    
    static let bodyPrimary = TypographyStyle(
        font: .bodyMedium,
        color: .textPrimary,
        lineSpacing: TextSpacing.normal
    )
    
    static let captionSecondary = TypographyStyle(
        font: .caption,
        color: .textSecondary,
        lineSpacing: TextSpacing.tight
    )
}

extension View {
    func typography(_ style: TypographyStyle) -> some View {
        self.modifier(TypographyModifier(style: style))
    }
}

