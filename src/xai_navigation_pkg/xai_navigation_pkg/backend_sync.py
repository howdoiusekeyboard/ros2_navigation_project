#!/usr/bin/env python3
"""
Backend Sync - Periodically syncs local decisions to backend API.

Week 3: Bridges local ROS2 database with unified backend.

Features:
- Background thread for periodic sync
- Batch sync of unsynced decisions
- Retry on failure
- Health check
"""

import time
import threading
from typing import Optional, Dict, Any, List

try:
    import requests
except ImportError:
    requests = None  # Will fail gracefully if not available


class BackendSync:
    """
    Syncs local navigation decisions to backend API.

    Runs as a background thread, periodically pushing unsynced
    decisions to the backend for unified storage and dashboard access.
    """

    def __init__(
        self,
        backend_url: str = 'http://localhost:8000',
        sync_interval: float = 5.0,
        batch_size: int = 50
    ):
        """
        Initialize backend sync service.

        Args:
            backend_url: Backend API base URL
            sync_interval: Seconds between sync attempts
            batch_size: Max decisions per sync batch
        """
        self.backend_url = backend_url.rstrip('/')
        self.sync_interval = sync_interval
        self.batch_size = batch_size

        self._decision_db = None  # Set by XAI Navigator
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Statistics
        self.stats = {
            'total_synced': 0,
            'sync_errors': 0,
            'last_sync_time': None,
            'last_error': None
        }

    def set_database(self, db):
        """
        Set the local database reference.

        Args:
            db: DecisionDatabase instance
        """
        self._decision_db = db

    def start(self):
        """Start background sync thread."""
        if self._running:
            return

        if requests is None:
            # Can't sync without requests library
            return

        self._running = True
        self._thread = threading.Thread(target=self._sync_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop background sync thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _sync_loop(self):
        """Background sync loop."""
        while self._running:
            try:
                self._do_sync()
            except Exception as e:
                self.stats['sync_errors'] += 1
                self.stats['last_error'] = str(e)

            time.sleep(self.sync_interval)

    def _do_sync(self):
        """Perform a single sync operation."""
        if not self._decision_db:
            return

        # Get unsynced decisions
        unsynced = self._decision_db.get_unsynced_decisions(self.batch_size)

        if not unsynced:
            return

        # Send to backend
        url = f'{self.backend_url}/api/v1/navigation/decisions/sync'

        try:
            response = requests.post(
                url,
                json=unsynced,
                timeout=10.0
            )

            if response.status_code == 200:
                # Mark as synced
                db_ids = [d['db_id'] for d in unsynced]
                self._decision_db.mark_synced(db_ids)

                self.stats['total_synced'] += len(db_ids)
                self.stats['last_sync_time'] = time.time()
            else:
                self.stats['sync_errors'] += 1
                self.stats['last_error'] = f'HTTP {response.status_code}'

        except Exception as e:
            self.stats['sync_errors'] += 1
            self.stats['last_error'] = str(e)

    def sync_now(self) -> Dict[str, Any]:
        """Force immediate sync and return result."""
        if not self._decision_db:
            return {'success': False, 'error': 'No database configured'}

        if requests is None:
            return {'success': False, 'error': 'requests library not available'}

        try:
            unsynced = self._decision_db.get_unsynced_decisions(self.batch_size)

            if not unsynced:
                return {'success': True, 'synced': 0, 'message': 'Nothing to sync'}

            url = f'{self.backend_url}/api/v1/navigation/decisions/sync'
            response = requests.post(url, json=unsynced, timeout=10.0)

            if response.status_code == 200:
                db_ids = [d['db_id'] for d in unsynced]
                self._decision_db.mark_synced(db_ids)
                self.stats['total_synced'] += len(db_ids)
                return {'success': True, 'synced': len(db_ids)}
            else:
                return {'success': False, 'error': f'HTTP {response.status_code}'}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def check_backend_health(self) -> bool:
        """Check if backend is reachable."""
        if requests is None:
            return False

        try:
            response = requests.get(
                f'{self.backend_url}/health',
                timeout=5.0
            )
            return response.status_code == 200
        except:
            return False

    def get_statistics(self) -> Dict[str, Any]:
        """Get sync statistics."""
        return {
            **self.stats,
            'backend_url': self.backend_url,
            'sync_interval': self.sync_interval,
            'is_running': self._running
        }
