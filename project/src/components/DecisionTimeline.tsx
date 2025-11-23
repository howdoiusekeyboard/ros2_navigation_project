import React, { useEffect, useState } from 'react';
import { History } from 'lucide-react';
import { rosService, Explanation } from '../services/rosService';

export const DecisionTimeline: React.FC = () => {
    const [history, setHistory] = useState<Explanation[]>([]);

    useEffect(() => {
        rosService.onExplanationReceived((newExplanation) => {
            setHistory((prev) => [newExplanation, ...prev].slice(0, 20)); // Keep last 20
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
            </div>

            <div className="flex-1 overflow-y-auto pr-2 space-y-4 custom-scrollbar">
                {history.map((item, index) => (
                    <div key={index} className="relative pl-4 border-l-2 border-indigo-100 hover:border-indigo-300 transition-colors">
                        <div className="absolute -left-[5px] top-0 w-2.5 h-2.5 rounded-full bg-indigo-500 ring-4 ring-white" />

                        <div className="flex items-center justify-between mb-1">
                            <span className="text-xs font-medium text-indigo-600 bg-indigo-50 px-2 py-0.5 rounded-full">
                                {item.decision_type}
                            </span>
                            <span className="text-xs text-gray-400">
                                {new Date(item.timestamp * 1000).toLocaleTimeString()}
                            </span>
                        </div>

                        <p className="text-sm text-gray-600 line-clamp-2">
                            {item.text}
                        </p>
                    </div>
                ))}
            </div>
        </div>
    );
};
