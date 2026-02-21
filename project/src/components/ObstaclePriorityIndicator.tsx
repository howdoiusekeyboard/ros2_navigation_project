import React from 'react';
import { AlertTriangle, User, Car, Package, Square, HelpCircle } from 'lucide-react';

/**
 * Obstacle classification data from ROS topic
 */
interface ObstacleClassification {
    type: 'human' | 'vehicle' | 'dynamic' | 'furniture' | 'wall' | 'unknown';
    priority_weight: number;
    confidence: number;
    reasoning: string;
    zone_name?: string;
    velocity?: number;
    size?: number;
}

interface ObstacleData {
    x: number;
    y: number;
    distance: number;
    severity: string;
}

interface ObstaclePriorityIndicatorProps {
    obstacle?: ObstacleData;
    classification?: ObstacleClassification;
    compact?: boolean;
}

/**
 * Color configuration for each obstacle type (Tesla-style prioritization)
 */
const TYPE_CONFIG = {
    human: {
        color: 'bg-red-500',
        textColor: 'text-red-600',
        bgLight: 'bg-red-50',
        borderColor: 'border-red-200',
        icon: User,
        label: 'Person',
        pulse: true,
    },
    vehicle: {
        color: 'bg-orange-500',
        textColor: 'text-orange-600',
        bgLight: 'bg-orange-50',
        borderColor: 'border-orange-200',
        icon: Car,
        label: 'Vehicle',
        pulse: true,
    },
    dynamic: {
        color: 'bg-yellow-500',
        textColor: 'text-yellow-600',
        bgLight: 'bg-yellow-50',
        borderColor: 'border-yellow-200',
        icon: AlertTriangle,
        label: 'Moving Object',
        pulse: true,
    },
    furniture: {
        color: 'bg-blue-500',
        textColor: 'text-blue-600',
        bgLight: 'bg-blue-50',
        borderColor: 'border-blue-200',
        icon: Package,
        label: 'Furniture',
        pulse: false,
    },
    wall: {
        color: 'bg-gray-400',
        textColor: 'text-gray-600',
        bgLight: 'bg-gray-50',
        borderColor: 'border-gray-200',
        icon: Square,
        label: 'Wall',
        pulse: false,
    },
    unknown: {
        color: 'bg-gray-300',
        textColor: 'text-gray-500',
        bgLight: 'bg-gray-50',
        borderColor: 'border-gray-200',
        icon: HelpCircle,
        label: 'Unknown',
        pulse: false,
    },
};

/**
 * ObstaclePriorityIndicator - Shows obstacle type with color-coded priority
 *
 * Tesla-style prioritization:
 * 🔴 Red = Human (highest priority, weight 10.0)
 * 🟠 Orange = Vehicle (weight 5.0)
 * 🟡 Yellow = Dynamic/Moving (weight 3.0)
 * 🔵 Blue = Furniture (weight 2.0)
 * ⚪ Gray = Wall/Static (weight 1.0)
 */
export const ObstaclePriorityIndicator: React.FC<ObstaclePriorityIndicatorProps> = ({
    obstacle,
    classification,
    compact = false,
}) => {
    const type = classification?.type || 'unknown';
    const config = TYPE_CONFIG[type] || TYPE_CONFIG.unknown;
    const Icon = config.icon;

    if (!classification) {
        return null;
    }

    const priorityWeight = classification.priority_weight || 1.0;
    const confidence = classification.confidence || 0;

    // Compact mode - just badge
    if (compact) {
        return (
            <div
                className={`inline-flex items-center gap-1 px-2 py-0.5 rounded-full ${config.bgLight} ${config.borderColor} border`}
                title={classification.reasoning}
            >
                <div
                    className={`w-2 h-2 rounded-full ${config.color} ${
                        config.pulse ? 'animate-pulse' : ''
                    }`}
                />
                <span className={`text-xs font-medium ${config.textColor}`}>
                    {config.label}
                </span>
            </div>
        );
    }

    // Full mode with details
    return (
        <div
            className={`rounded-lg border ${config.borderColor} ${config.bgLight} p-3`}
        >
            {/* Header with type and priority */}
            <div className="flex items-center justify-between mb-2">
                <div className="flex items-center gap-2">
                    <div
                        className={`w-8 h-8 rounded-full ${config.color} flex items-center justify-center ${
                            config.pulse ? 'animate-pulse' : ''
                        }`}
                    >
                        <Icon className="w-4 h-4 text-white" />
                    </div>
                    <div>
                        <div className={`font-semibold ${config.textColor}`}>
                            {config.label}
                        </div>
                        {classification.zone_name && (
                            <div className="text-xs text-gray-500">
                                in {classification.zone_name}
                            </div>
                        )}
                    </div>
                </div>

                {/* Priority badge */}
                <div className="text-right">
                    <div
                        className={`text-lg font-bold ${config.textColor}`}
                    >
                        {priorityWeight.toFixed(1)}
                    </div>
                    <div className="text-xs text-gray-400">Priority</div>
                </div>
            </div>

            {/* Distance and metrics */}
            {obstacle && (
                <div className="grid grid-cols-3 gap-2 mb-2 text-center">
                    <div className="bg-white/50 rounded p-1">
                        <div className="text-sm font-medium text-gray-700">
                            {obstacle.distance.toFixed(2)}m
                        </div>
                        <div className="text-xs text-gray-400">Distance</div>
                    </div>
                    <div className="bg-white/50 rounded p-1">
                        <div className="text-sm font-medium text-gray-700">
                            {(confidence * 100).toFixed(0)}%
                        </div>
                        <div className="text-xs text-gray-400">Confidence</div>
                    </div>
                    <div className="bg-white/50 rounded p-1">
                        <div className="text-sm font-medium text-gray-700">
                            {classification.velocity
                                ? `${classification.velocity.toFixed(1)} m/s`
                                : 'Static'}
                        </div>
                        <div className="text-xs text-gray-400">Velocity</div>
                    </div>
                </div>
            )}

            {/* Reasoning tooltip */}
            {classification.reasoning && (
                <div className="text-xs text-gray-600 italic border-t border-gray-200/50 pt-2 mt-2">
                    {classification.reasoning}
                </div>
            )}
        </div>
    );
};

/**
 * PriorityBadge - Inline badge showing obstacle type
 */
export const PriorityBadge: React.FC<{
    type: string;
    priority: number;
}> = ({ type, priority }) => {
    const config = TYPE_CONFIG[type as keyof typeof TYPE_CONFIG] || TYPE_CONFIG.unknown;

    return (
        <div
            className={`inline-flex items-center gap-1.5 px-2 py-1 rounded-md ${config.bgLight} ${config.borderColor} border`}
        >
            <div
                className={`w-2.5 h-2.5 rounded-full ${config.color} ${
                    config.pulse ? 'animate-pulse' : ''
                }`}
            />
            <span className={`text-xs font-medium ${config.textColor}`}>
                {config.label}
            </span>
            <span className="text-xs text-gray-400">P:{priority.toFixed(1)}</span>
        </div>
    );
};

export default ObstaclePriorityIndicator;
