import React, { useEffect, useState } from 'react';
import { History, User, Car, Package, Square, AlertTriangle } from 'lucide-react';
import { rosService, Explanation } from '../services/rosService';

/**
 * Extended explanation with classification data
 */
interface ExtendedExplanation extends Explanation {
    obstacle_type?: string;
    priority_weight?: number;
    classification_confidence?: number;
}

/**
 * Color configuration for obstacle types (Tesla-style prioritization)
 */
const TYPE_COLORS: Record<string, { dot: string; bg: string; text: string; icon: React.ElementType }> = {
    human: { dot: 'bg-red-500', bg: 'bg-red-50', text: 'text-red-600', icon: User },
    vehicle: { dot: 'bg-orange-500', bg: 'bg-orange-50', text: 'text-orange-600', icon: Car },
    dynamic: { dot: 'bg-yellow-500', bg: 'bg-yellow-50', text: 'text-yellow-600', icon: AlertTriangle },
    furniture: { dot: 'bg-blue-500', bg: 'bg-blue-50', text: 'text-blue-600', icon: Package },
    wall: { dot: 'bg-gray-400', bg: 'bg-gray-50', text: 'text-gray-600', icon: Square },
    unknown: { dot: 'bg-gray-300', bg: 'bg-gray-50', text: 'text-gray-500', icon: AlertTriangle },
};

/**
 * Decision type colors (existing)
 */
const DECISION_TYPE_COLORS: Record<string, string> = {
    obstacle_detected: 'bg-red-100 text-red-700',
    path_changed: 'bg-amber-100 text-amber-700',
    goal_reached: 'bg-green-100 text-green-700',
    goal_received: 'bg-blue-100 text-blue-700',
    goal_aborted: 'bg-red-100 text-red-700',
    feedback: 'bg-gray-100 text-gray-600',
};

export const DecisionTimeline: React.FC = () => {
    const [history, setHistory] = useState<ExtendedExplanation[]>([]);

    useEffect(() => {
        rosService.onExplanationReceived((newExplanation) => {
            setHistory((prev) => [newExplanation as ExtendedExplanation, ...prev].slice(0, 20)); // Keep last 20
        });
    }, []);

    if (history.length === 0) {
        return (
            <div className="bg-white rounded-xl shadow-sm p-6 border border-gray-100 h-full flex flex-col items-center justify-center text-gray-400">
                <History className="w-12 h-12 mb-3 opacity-20" />
                <p>No decision history yet</p>
            </div>
        );
    }

    return (
        <div className="bg-white rounded-xl shadow-sm p-6 border border-gray-100 h-full flex flex-col">
            <div className="flex items-center gap-2 mb-4">
                <History className="w-5 h-5 text-indigo-600" />
                <h2 className="text-lg font-semibold text-gray-800">Decision Timeline</h2>
                <span className="ml-auto text-xs text-gray-400">{history.length} events</span>
            </div>

            <div className="flex-1 overflow-y-auto pr-2 space-y-4 custom-scrollbar">
                {history.map((item, index) => {
                    // Determine if this is an obstacle event with classification
                    const hasClassification = item.decision_type === 'obstacle_detected' && item.obstacle_type;
                    const obstacleType = item.obstacle_type || 'unknown';
                    const typeConfig = TYPE_COLORS[obstacleType] || TYPE_COLORS.unknown;
                    const TypeIcon = typeConfig.icon;

                    // Border color based on obstacle type for obstacle events
                    const borderClass = hasClassification
                        ? `border-l-4 ${typeConfig.dot.replace('bg-', 'border-')}`
                        : 'border-l-2 border-indigo-100 hover:border-indigo-300';

                    return (
                        <div
                            key={index}
                            className={`relative pl-4 ${borderClass} transition-colors`}
                        >
                            {/* Timeline dot - color-coded for obstacle types */}
                            <div
                                className={`absolute -left-[7px] top-0 w-3 h-3 rounded-full ring-4 ring-white ${
                                    hasClassification ? typeConfig.dot : 'bg-indigo-500'
                                } ${hasClassification && (obstacleType === 'human' || obstacleType === 'vehicle') ? 'animate-pulse' : ''}`}
                            />

                            {/* Decision type badge and timestamp */}
                            <div className="flex items-center justify-between mb-1 flex-wrap gap-1">
                                <span
                                    className={`text-xs font-medium px-2 py-0.5 rounded-full ${
                                        DECISION_TYPE_COLORS[item.decision_type] || 'bg-gray-100 text-gray-600'
                                    }`}
                                >
                                    {item.decision_type}
                                </span>

                                {/* Obstacle type badge with priority */}
                                {hasClassification && (
                                    <span
                                        className={`inline-flex items-center gap-1 text-xs font-medium px-2 py-0.5 rounded-full ${typeConfig.bg} ${typeConfig.text}`}
                                        title={`Priority: ${item.priority_weight?.toFixed(1) || '?'}`}
                                    >
                                        <TypeIcon className="w-3 h-3" />
                                        {obstacleType.charAt(0).toUpperCase() + obstacleType.slice(1)}
                                        {item.priority_weight && (
                                            <span className="opacity-70 ml-1">
                                                P:{item.priority_weight.toFixed(1)}
                                            </span>
                                        )}
                                    </span>
                                )}

                                <span className="text-xs text-gray-400">
                                    {new Date(item.timestamp * 1000).toLocaleTimeString()}
                                </span>
                            </div>

                            {/* Explanation text */}
                            <p className="text-sm text-gray-600 line-clamp-2">
                                {item.text}
                            </p>

                            {/* Confidence indicator for classifications */}
                            {hasClassification && item.classification_confidence !== undefined && (
                                <div className="mt-1 flex items-center gap-2">
                                    <div className="flex-1 h-1 bg-gray-100 rounded-full overflow-hidden">
                                        <div
                                            className={`h-full ${typeConfig.dot}`}
                                            style={{ width: `${(item.classification_confidence || 0) * 100}%` }}
                                        />
                                    </div>
                                    <span className="text-xs text-gray-400">
                                        {((item.classification_confidence || 0) * 100).toFixed(0)}% conf.
                                    </span>
                                </div>
                            )}
                        </div>
                    );
                })}
            </div>

            {/* Legend for obstacle types */}
            <div className="mt-4 pt-3 border-t border-gray-100">
                <div className="flex flex-wrap gap-2 text-xs">
                    {Object.entries(TYPE_COLORS).slice(0, 5).map(([type, config]) => (
                        <div key={type} className="flex items-center gap-1">
                            <div className={`w-2 h-2 rounded-full ${config.dot}`} />
                            <span className="text-gray-500 capitalize">{type}</span>
                        </div>
                    ))}
                </div>
            </div>
        </div>
    );
};
