/**
 * Conversation Memory Service
 *
 * Handles conversation history and session management for Week 2 implementation.
 * Provides methods for:
 * - Retrieving conversation history
 * - Managing sessions
 * - Accessing spatial references
 * - Session persistence in localStorage
 */

export interface ConversationTurn {
  id: number;
  timestamp: string;
  turn_number: number;
  user_input: string;
  robot_response: string;
  action_type: string;
  location: {
    x: number;
    y: number;
    label: string;
  } | null;
  confidence: number;
  latency_ms: number;
  metadata?: any;
}

export interface ConversationHistory {
  session_id: string;
  history: ConversationTurn[];
  turn_count: number;
}

export interface SessionSummary {
  session_id: string;
  turn_count: number;
  start_time: string;
  end_time: string;
  avg_confidence: number;
  avg_latency_ms: number;
  navigation_count: number;
}

export interface SpatialReference {
  x: number;
  y: number;
}

export interface SpatialReferences {
  session_id: string;
  references: Record<string, SpatialReference>;
}

export interface ConversationServiceConfig {
  baseUrl?: string;
  timeout?: number;
  sessionStorageKey?: string;
}

class ConversationService {
  private baseUrl: string;
  private timeout: number;
  private sessionStorageKey: string;
  private currentSessionId: string | null;

  constructor(config?: ConversationServiceConfig) {
    this.baseUrl = config?.baseUrl || import.meta.env.VITE_BACKEND_URL || 'http://localhost:8000';
    this.timeout = config?.timeout || 10000; // 10 seconds
    // IMPORTANT: Use same key as backendService for session ID consistency
    this.sessionStorageKey = config?.sessionStorageKey || 'conversation_session_id';

    // Load session ID from localStorage
    this.currentSessionId = this.loadSessionFromStorage();
  }

  /**
   * Get current session ID or create a new one
   */
  public getCurrentSessionId(): string | null {
    return this.currentSessionId;
  }

  /**
   * Set current session ID and persist to localStorage
   */
  public setCurrentSessionId(sessionId: string): void {
    this.currentSessionId = sessionId;
    this.saveSessionToStorage(sessionId);
  }

  /**
   * Clear current session
   */
  public clearCurrentSession(): void {
    this.currentSessionId = null;
    localStorage.removeItem(this.sessionStorageKey);
  }

  /**
   * Load session ID from localStorage
   */
  private loadSessionFromStorage(): string | null {
    try {
      return localStorage.getItem(this.sessionStorageKey);
    } catch (error) {
      console.warn('Failed to load session from localStorage:', error);
      return null;
    }
  }

  /**
   * Save session ID to localStorage
   */
  private saveSessionToStorage(sessionId: string): void {
    try {
      localStorage.setItem(this.sessionStorageKey, sessionId);
    } catch (error) {
      console.error('Failed to save session to localStorage:', error);
    }
  }

  /**
   * Get conversation history for a session
   */
  public async getConversationHistory(
    sessionId?: string,
    limit: number = 20,
    includeMetadata: boolean = false
  ): Promise<ConversationHistory> {
    const sid = sessionId || this.currentSessionId;

    if (!sid) {
      throw new Error('No session ID available');
    }

    const params = new URLSearchParams({
      limit: limit.toString(),
      include_metadata: includeMetadata.toString()
    });

    const response = await fetch(
      `${this.baseUrl}/api/v1/conversation/history/${sid}?${params}`,
      {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: AbortSignal.timeout(this.timeout),
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to fetch conversation history: ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Get list of recent sessions
   */
  public async getRecentSessions(limit: number = 10): Promise<SessionSummary[]> {
    const params = new URLSearchParams({
      limit: limit.toString()
    });

    const response = await fetch(
      `${this.baseUrl}/api/v1/conversation/sessions?${params}`,
      {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: AbortSignal.timeout(this.timeout),
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to fetch sessions: ${response.statusText}`);
    }

    const data = await response.json();
    return data.sessions;
  }

  /**
   * Get session summary statistics
   */
  public async getSessionSummary(sessionId?: string): Promise<SessionSummary> {
    const sid = sessionId || this.currentSessionId;

    if (!sid) {
      throw new Error('No session ID available');
    }

    const response = await fetch(
      `${this.baseUrl}/api/v1/conversation/summary/${sid}`,
      {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: AbortSignal.timeout(this.timeout),
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to fetch session summary: ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Delete a conversation session (soft delete)
   */
  public async deleteSession(sessionId?: string): Promise<{ message: string; deleted: boolean }> {
    const sid = sessionId || this.currentSessionId;

    if (!sid) {
      throw new Error('No session ID available');
    }

    const response = await fetch(
      `${this.baseUrl}/api/v1/conversation/session/${sid}`,
      {
        method: 'DELETE',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: AbortSignal.timeout(this.timeout),
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to delete session: ${response.statusText}`);
    }

    // If we deleted the current session, clear it
    if (sid === this.currentSessionId) {
      this.clearCurrentSession();
    }

    return await response.json();
  }

  /**
   * Get spatial references (labeled locations) for a session
   */
  public async getSpatialReferences(sessionId?: string): Promise<SpatialReferences> {
    const sid = sessionId || this.currentSessionId;

    if (!sid) {
      throw new Error('No session ID available');
    }

    const response = await fetch(
      `${this.baseUrl}/api/v1/conversation/spatial_refs/${sid}`,
      {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: AbortSignal.timeout(this.timeout),
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to fetch spatial references: ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Format timestamp for display
   */
  public formatTimestamp(timestamp: string): string {
    try {
      const date = new Date(timestamp);
      return date.toLocaleString('en-US', {
        month: 'short',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit',
      });
    } catch (error) {
      return timestamp;
    }
  }

  /**
   * Format duration from milliseconds to human-readable string
   */
  public formatDuration(ms: number): string {
    if (ms < 1000) {
      return `${Math.round(ms)}ms`;
    } else {
      return `${(ms / 1000).toFixed(2)}s`;
    }
  }

  /**
   * Get confidence color class based on confidence score
   */
  public getConfidenceColorClass(confidence: number): string {
    if (confidence >= 0.9) return 'text-green-600';
    if (confidence >= 0.7) return 'text-yellow-600';
    return 'text-red-600';
  }

  /**
   * Get action type display name
   */
  public getActionTypeDisplay(actionType: string): string {
    const displayNames: Record<string, string> = {
      'twist': 'Velocity Control',
      'navigate': 'Navigation',
      'stop': 'Stop',
      'rotate': 'Rotation',
      'move_forward': 'Move Forward',
      'move_backward': 'Move Backward',
      'unknown': 'Unknown',
    };
    return displayNames[actionType] || actionType;
  }

  /**
   * Check if conversation history is available for current session
   */
  public async hasConversationHistory(): Promise<boolean> {
    if (!this.currentSessionId) {
      return false;
    }

    try {
      const history = await this.getConversationHistory(this.currentSessionId, 1);
      return history.turn_count > 0;
    } catch (error) {
      return false;
    }
  }
}

// Export singleton instance
export const conversationService = new ConversationService();

// Export class for custom instances
export default ConversationService;
