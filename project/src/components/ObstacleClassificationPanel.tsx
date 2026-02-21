import React, { useState, useEffect, useCallback } from 'react';
import {
    AlertTriangle,
    User,
    ShoppingCart,
    Package,
    Square,
    HelpCircle,
    Eye,
    Radio,
    Zap,
    ArrowRight,
    ArrowUpRight,
    ArrowDownRight,
    Minus,
    RefreshCw,
    Camera,
    Radar
} from 'lucide-react';

/**
 * Tesla-Style Obstacle Classification Panel
 *
 * Displays real-time obstacle detections with:
 * - Priority-based color coding (Human > Vehicle > Furniture > Wall)
 * - Detection source indicator (Camera vs LiDAR)
 * - Intent visualization (approaching, crossing, stationary, moving_away)
 * - Time-to-collision countdown for critical obstacles
 */

interface FusedObstacle {
    x: number;
    y: number;
    distance: number;
    obstacle_type: 'human' | 'vehicle' | 'dynamic' | 'furniture' | 'wall' | 'unknown';
    priority_weight: number;
    confidence: number;
    detection_source: 'camera' | 'lidar_heuristic' | 'fusion';
    camera_class?: string;
    camera_confidence?: number;
    intent: 'approaching' | 'crossing' | 'stationary' | 'moving_away';
    collision_risk: number;
    time_to_collision?: number;
    reasoning: string;
    contributing_factors: string[];
    velocity?: number;
}

interface ObstacleClassificationPanelProps {
    obstacles?: FusedObstacle[];
    robotPose?: { x: number; y: number; theta: number };
    isConnected?: boolean;
    onObstacleSelect?: (obstacle: FusedObstacle) => void;
}

// Tesla-style type configuration
const TYPE_CONFIG = {
    human: {
        color: 'bg-red-500',
        textColor: 'text-red-600',
        bgLight: 'bg-red-50',
        borderColor: 'border-red-300',
        ringColor: 'ring-red-400',
        icon: User,
        label: 'Person',
        description: 'Highest safety priority',
        pulse: true,
    },
    vehicle: {
        color: 'bg-orange-500',
        textColor: 'text-orange-600',
        bgLight: 'bg-orange-50',
        borderColor: 'border-orange-300',
        ringColor: 'ring-orange-400',
        icon: ShoppingCart,
        label: 'Vehicle',
        description: 'Cart/wheelchair/scooter',
        pulse: true,
    },
    dynamic: {
        color: 'bg-yellow-500',
        textColor: 'text-yellow-700',
        bgLight: 'bg-yellow-50',
        borderColor: 'border-yellow-300',
        ringColor: 'ring-yellow-400',
        icon: Zap,
        label: 'Dynamic',
        description: 'Moving unknown object',
        pulse: true,
    },
    furniture: {
        color: 'bg-blue-500',
        textColor: 'text-blue-600',
        bgLight: 'bg-blue-50',
        borderColor: 'border-blue-200',
        ringColor: 'ring-blue-400',
        icon: Package,
        label: 'Furniture',
        description: 'Chair/table/desk',
        pulse: false,
    },
    wall: {
        color: 'bg-gray-400',
        textColor: 'text-gray-600',
        bgLight: 'bg-gray-50',
        borderColor: 'border-gray-200',
        ringColor: 'ring-gray-300',
        icon: Square,
        label: 'Wall',
        description: 'Static structure',
        pulse: false,
    },
    unknown: {
        color: 'bg-gray-300',
        textColor: 'text-gray-500',
        bgLight: 'bg-gray-50',
        borderColor: 'border-gray-200',
        ringColor: 'ring-gray-300',
        icon: HelpCircle,
        label: 'Unknown',
        description: 'Unclassified obstacle',
        pulse: false,
    },
};

// Intent icons
const INTENT_ICONS = {
    approaching: ArrowUpRight,
    crossing: ArrowRight,
    stationary: Minus,
    moving_away: ArrowDownRight,
};

const INTENT_LABELS = {
    approaching: 'Approaching',
    crossing: 'Crossing',
    stationary: 'Stationary',
    moving_away: 'Moving Away',
};

/**
 * Single Obstacle Card
 */
const ObstacleCard: React.FC<{
    obstacle: FusedObstacle;
    isHighestPriority: boolean;
    onSelect?: (obstacle: FusedObstacle) => void;
}> = ({ obstacle, isHighestPriority, onSelect }) => {
    const config = TYPE_CONFIG[obstacle.obstacle_type] || TYPE_CONFIG.unknown;
    const TypeIcon = config.icon;
    const IntentIcon = INTENT_ICONS[obstacle.intent] || Minus;

    const isUrgent = obstacle.collision_risk > 0.5 ||
                     (obstacle.time_to_collision !== undefined && obstacle.time_to_collision < 3);

    return (
        <div
            className={`
                rounded-lg border-2 p-3 transition-all cursor-pointer
                ${config.borderColor} ${config.bgLight}
                ${isHighestPriority ? `ring-2 ${config.ringColor} shadow-md` : ''}
                ${isUrgent ? 'animate-pulse' : ''}
                hover:shadow-lg hover:scale-[1.02]
            `}
            onClick={() => onSelect?.(obstacle)}
        >
            {/* Header: Type + Priority */}
            <div className="flex items-start justify-between mb-2">
                <div className="flex items-center gap-2">
                    <div className={`
                        w-10 h-10 rounded-full flex items-center justify-center
                        ${config.color} ${config.pulse ? 'animate-pulse' : ''}
                    `}>
                        <TypeIcon className="w-5 h-5 text-white" />
                    </div>
                    <div>
                        <div className={`font-bold ${config.textColor}`}>
                            {config.label}
                            {isHighestPriority && (
                                <span className="ml-2 text-xs bg-red-100 text-red-700 px-1.5 py-0.5 rounded">
                                    PRIORITY
                                </span>
                            )}
                        </div>
                        <div className="text-xs text-gray-500">
                            {obstacle.camera_class || config.description}
                        </div>
                    </div>
                </div>

                {/* Priority Weight Badge */}
                <div className="text-right">
                    <div className={`text-xl font-bold ${config.textColor}`}>
                        {obstacle.priority_weight.toFixed(1)}
                    </div>
                    <div className="text-xs text-gray-400">Priority</div>
                </div>
            </div>

            {/* Metrics Grid */}
            <div className="grid grid-cols-4 gap-1 text-center mb-2">
                {/* Distance */}
                <div className="bg-white/60 rounded p-1">
                    <div className="text-sm font-semibold text-gray-800">
                        {obstacle.distance.toFixed(1)}m
                    </div>
                    <div className="text-[10px] text-gray-400">Distance</div>
                </div>

                {/* Confidence */}
                <div className="bg-white/60 rounded p-1">
                    <div className="text-sm font-semibold text-gray-800">
                        {(obstacle.confidence * 100).toFixed(0)}%
                    </div>
                    <div className="text-[10px] text-gray-400">Conf.</div>
                </div>

                {/* Intent */}
                <div className={`rounded p-1 ${
                    obstacle.intent === 'approaching' ? 'bg-red-100' :
                    obstacle.intent === 'crossing' ? 'bg-yellow-100' :
                    'bg-white/60'
                }`}>
                    <div className="flex items-center justify-center">
                        <IntentIcon className={`w-4 h-4 ${
                            obstacle.intent === 'approaching' ? 'text-red-600' :
                            obstacle.intent === 'crossing' ? 'text-yellow-700' :
                            'text-gray-600'
                        }`} />
                    </div>
                    <div className="text-[10px] text-gray-500">
                        {INTENT_LABELS[obstacle.intent]}
                    </div>
                </div>

                {/* Time to Collision / Velocity */}
                <div className={`rounded p-1 ${
                    obstacle.time_to_collision !== undefined && obstacle.time_to_collision < 3
                        ? 'bg-red-200 animate-pulse'
                        : 'bg-white/60'
                }`}>
                    {obstacle.time_to_collision !== undefined ? (
                        <>
                            <div className="text-sm font-bold text-red-700">
                                {obstacle.time_to_collision.toFixed(1)}s
                            </div>
                            <div className="text-[10px] text-red-500">TTC</div>
                        </>
                    ) : (
                        <>
                            <div className="text-sm font-semibold text-gray-800">
                                {obstacle.velocity ? `${obstacle.velocity.toFixed(1)}` : '0.0'}
                            </div>
                            <div className="text-[10px] text-gray-400">m/s</div>
                        </>
                    )}
                </div>
            </div>

            {/* Detection Source */}
            <div className="flex items-center justify-between text-xs text-gray-500 border-t border-gray-200/50 pt-2">
                <div className="flex items-center gap-1">
                    {obstacle.detection_source === 'camera' ? (
                        <Camera className="w-3 h-3 text-blue-500" />
                    ) : obstacle.detection_source === 'fusion' ? (
                        <><Camera className="w-3 h-3 text-blue-500" /><Radar className="w-3 h-3 text-green-500" /></>
                    ) : (
                        <Radar className="w-3 h-3 text-green-500" />
                    )}
                    <span>
                        {obstacle.detection_source === 'fusion' ? 'Camera + LiDAR' :
                         obstacle.detection_source === 'camera' ? 'Camera' : 'LiDAR'}
                    </span>
                </div>

                {/* Collision Risk Bar */}
                {obstacle.collision_risk > 0 && (
                    <div className="flex items-center gap-1">
                        <div className="w-16 h-1.5 bg-gray-200 rounded-full overflow-hidden">
                            <div
                                className={`h-full rounded-full ${
                                    obstacle.collision_risk > 0.7 ? 'bg-red-500' :
                                    obstacle.collision_risk > 0.3 ? 'bg-yellow-500' :
                                    'bg-green-500'
                                }`}
                                style={{ width: `${obstacle.collision_risk * 100}%` }}
                            />
                        </div>
                        <span className="text-[10px]">
                            {(obstacle.collision_risk * 100).toFixed(0)}% risk
                        </span>
                    </div>
                )}
            </div>
        </div>
    );
};

/**
 * Priority Legend
 */
const PriorityLegend: React.FC = () => (
    <div className="flex flex-wrap gap-2 text-xs">
        {Object.entries(TYPE_CONFIG).map(([type, config]) => (
            <div
                key={type}
                className={`flex items-center gap-1 px-2 py-0.5 rounded-full ${config.bgLight} ${config.borderColor} border`}
            >
                <div className={`w-2 h-2 rounded-full ${config.color}`} />
                <span className={config.textColor}>{config.label}</span>
            </div>
        ))}
    </div>
);

/**
 * Main Panel Component
 */
export const ObstacleClassificationPanel: React.FC<ObstacleClassificationPanelProps> = ({
    obstacles = [],
    robotPose,
    isConnected = false,
    onObstacleSelect,
}) => {
    const [selectedObstacle, setSelectedObstacle] = useState<FusedObstacle | null>(null);

    // Sort obstacles by priority (highest first)
    const sortedObstacles = [...obstacles].sort(
        (a, b) => b.priority_weight - a.priority_weight
    );

    const highestPriority = sortedObstacles[0];

    // Count by type
    const typeCounts = obstacles.reduce((acc, obs) => {
        acc[obs.obstacle_type] = (acc[obs.obstacle_type] || 0) + 1;
        return acc;
    }, {} as Record<string, number>);

    const handleSelect = useCallback((obstacle: FusedObstacle) => {
        setSelectedObstacle(obstacle);
        onObstacleSelect?.(obstacle);
    }, [onObstacleSelect]);

    return (
        <div className="bg-white rounded-xl shadow-lg border border-gray-200 overflow-hidden">
            {/* Header */}
            <div className="bg-gradient-to-r from-slate-800 to-slate-700 px-4 py-3">
                <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                        <Eye className="w-5 h-5 text-blue-400" />
                        <h2 className="text-white font-semibold">
                            Obstacle Classification
                        </h2>
                        <span className="bg-slate-600 text-slate-200 text-xs px-2 py-0.5 rounded-full">
                            Tesla-Style
                        </span>
                    </div>
                    <div className="flex items-center gap-3">
                        {/* Connection Status */}
                        <div className={`flex items-center gap-1 text-xs ${
                            isConnected ? 'text-green-400' : 'text-red-400'
                        }`}>
                            <Radio className={`w-3 h-3 ${isConnected ? 'animate-pulse' : ''}`} />
                            {isConnected ? 'Live' : 'Disconnected'}
                        </div>

                        {/* Obstacle Count */}
                        <div className="bg-slate-600 text-white text-sm px-2 py-0.5 rounded-full">
                            {obstacles.length} obstacles
                        </div>
                    </div>
                </div>

                {/* Type Distribution Bar */}
                {obstacles.length > 0 && (
                    <div className="mt-2 flex gap-3 text-xs text-slate-300">
                        {Object.entries(typeCounts).map(([type, count]) => {
                            const config = TYPE_CONFIG[type as keyof typeof TYPE_CONFIG];
                            return (
                                <div key={type} className="flex items-center gap-1">
                                    <div className={`w-2 h-2 rounded-full ${config?.color || 'bg-gray-500'}`} />
                                    <span>{count} {config?.label || type}</span>
                                </div>
                            );
                        })}
                    </div>
                )}
            </div>

            {/* Legend */}
            <div className="px-4 py-2 bg-gray-50 border-b border-gray-200">
                <PriorityLegend />
            </div>

            {/* Obstacles List */}
            <div className="p-4 max-h-[500px] overflow-y-auto">
                {obstacles.length === 0 ? (
                    <div className="text-center py-8 text-gray-400">
                        <HelpCircle className="w-12 h-12 mx-auto mb-2 opacity-50" />
                        <p>No obstacles detected</p>
                        <p className="text-xs mt-1">
                            Waiting for camera and LiDAR data...
                        </p>
                    </div>
                ) : (
                    <div className="grid gap-3">
                        {sortedObstacles.map((obstacle, index) => (
                            <ObstacleCard
                                key={`${obstacle.x.toFixed(1)}_${obstacle.y.toFixed(1)}_${index}`}
                                obstacle={obstacle}
                                isHighestPriority={obstacle === highestPriority}
                                onSelect={handleSelect}
                            />
                        ))}
                    </div>
                )}
            </div>

            {/* Footer with Summary */}
            {obstacles.length > 0 && highestPriority && (
                <div className={`
                    px-4 py-3 border-t border-gray-200
                    ${highestPriority.obstacle_type === 'human' ? 'bg-red-50' :
                      highestPriority.obstacle_type === 'vehicle' ? 'bg-orange-50' :
                      'bg-gray-50'}
                `}>
                    <div className="flex items-center justify-between text-sm">
                        <div className="flex items-center gap-2">
                            <AlertTriangle className={`w-4 h-4 ${
                                TYPE_CONFIG[highestPriority.obstacle_type]?.textColor || 'text-gray-600'
                            }`} />
                            <span className="font-medium">
                                Highest Priority: {TYPE_CONFIG[highestPriority.obstacle_type]?.label}
                            </span>
                            <span className="text-gray-500">
                                at {highestPriority.distance.toFixed(1)}m
                            </span>
                        </div>
                        <span className={`font-bold ${
                            TYPE_CONFIG[highestPriority.obstacle_type]?.textColor || 'text-gray-600'
                        }`}>
                            Weight: {highestPriority.priority_weight.toFixed(1)}
                        </span>
                    </div>
                </div>
            )}
        </div>
    );
};

export default ObstacleClassificationPanel;
