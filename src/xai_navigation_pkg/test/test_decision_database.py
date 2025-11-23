import unittest
import os
import shutil
import tempfile
import json
from xai_navigation_pkg.decision_database import DecisionDatabase

class TestDecisionDatabase(unittest.TestCase):
    def setUp(self):
        # Create a temporary directory for the database
        self.test_dir = tempfile.mkdtemp()
        self.db_path = os.path.join(self.test_dir, 'test_decisions.db')
        self.db = DecisionDatabase(self.db_path)

    def tearDown(self):
        self.db.close()
        shutil.rmtree(self.test_dir)

    def test_log_decision(self):
        decision = {
            'decision_id': 1,
            'decision_type': 'goal_sent',
            'timestamp': 1234567890.0,
            'goal': {'x': 1.0, 'y': 2.0},
            'data': {'current_x': 0.0, 'current_y': 0.0}
        }
        db_id = self.db.log_decision(decision, session_id='test_session')
        self.assertIsNotNone(db_id)
        
        # Verify retrieval
        decisions = self.db.get_recent_decisions(limit=1)
        self.assertEqual(len(decisions), 1)
        self.assertEqual(decisions[0]['decision_type'], 'goal_sent')
        self.assertEqual(decisions[0]['goal']['x'], 1.0)

    def test_unsynced_decisions(self):
        decision = {
            'decision_type': 'test',
            'data': {}
        }
        self.db.log_decision(decision)
        
        unsynced = self.db.get_unsynced_decisions()
        self.assertEqual(len(unsynced), 1)
        
        self.db.mark_synced([unsynced[0]['db_id']])
        
        unsynced_after = self.db.get_unsynced_decisions()
        self.assertEqual(len(unsynced_after), 0)

    def test_log_path_change(self):
        decision_id = 1
        self.db.log_path_change(decision_id, 10.0, 12.0, 0.5, 'obstacle')
        
        # Verify (would need a getter method or direct SQL check, 
        # but for now just ensuring no error on insert)
        pass

    def test_cleanup_old(self):
        # This is hard to test without mocking time or inserting old timestamps
        # But we can check it runs without error
        deleted = self.db.cleanup_old(days=1)
        self.assertEqual(deleted, 0)

if __name__ == '__main__':
    unittest.main()
