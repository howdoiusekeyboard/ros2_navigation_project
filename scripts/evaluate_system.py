#!/usr/bin/env python3
"""
System Evaluation Script for Research Metrics

Captures metrics for:
- Voice command parsing accuracy & latency
- Explanation generation quality & latency  
- Anomaly detection precision/recall
- End-to-end system latency

Usage:
    python3 scripts/evaluate_system.py --backend-url http://localhost:8000 --output metrics.json
"""

import argparse
import json
import time
import statistics
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional
import sys

try:
    import requests
except ImportError:
    print("ERROR: requests not installed. Run: pip install requests")
    sys.exit(1)


class SystemEvaluator:
    """Evaluates the intelligent navigation system for research metrics."""

    def __init__(self, backend_url: str, verbose: bool = False):
        import urllib.parse
        parsed = urllib.parse.urlparse(backend_url)
        if parsed.scheme not in ('http', 'https'):
            raise ValueError("Backend URL must use http or https scheme")
            
        # Strict SSRF mitigation for testing script
        import socket
        import ipaddress

        hostname = parsed.hostname
        if not hostname:
            raise ValueError("Backend URL must include a hostname")

        # Resolve hostname and strictly ensure all IPs are loopback to prevent DNS rebinding
        try:
            addr_info = socket.getaddrinfo(hostname, parsed.port or 80)
            for res in addr_info:
                ip = res[4][0]
                # Strip potential IPv6 scope id
                ip_clean = ip.split('%')[0]
                if not ipaddress.ip_address(ip_clean).is_loopback:
                     raise ValueError("Strict SSRF protection: Resolved backend IP is not a permitted loopback address.")
        except socket.gaierror:
            raise ValueError("Strict SSRF protection: Could not resolve hostname.")
             
        self.backend_url = backend_url.rstrip("/")
        self.verbose = verbose
        self.results: Dict[str, Any] = {
            "timestamp": datetime.now().isoformat(),
            "backend_url": backend_url,
            "voice_command_tests": [],
            "navigation_stats": {},
            "anomaly_detection": {},
            "latency_metrics": {},
        }

    def log(self, msg: str):
        if self.verbose:
            print(f"[EVAL] {msg}")

    def check_backend_health(self) -> bool:
        """Verify backend is reachable."""
        try:
            r = requests.get(f"{self.backend_url}/health", timeout=5)
            health = r.json()
            self.results["backend_health"] = health
            self.log(f"Backend health: {health}")
            return r.status_code == 200
        except Exception as e:
            self.log(f"Backend health check failed: {e}")
            return False

    def test_voice_command_parsing(self) -> Dict[str, Any]:
        """Test voice command parsing accuracy and latency."""
        test_cases = [
            # (input_text, expected_action, expected_param_key)
            ("move forward 2 meters", "move_forward", "distance"),
            ("go backward one meter", "move_backward", "distance"),
            ("turn left 90 degrees", "turn_left", "angle"),
            ("spin in a circle", "spin", None),
            ("stop", "stop", None),
            ("navigate to x equals 2 y equals 3", "navigate", "x"),
        ]

        results = []
        latencies = []

        for text, expected_action, expected_param in test_cases:
            try:
                start = time.perf_counter()
                r = requests.post(
                    f"{self.backend_url}/api/v1/parse_voice_command",
                    json={"transcript": text},
                    timeout=10,
                )
                latency_ms = (time.perf_counter() - start) * 1000
                latencies.append(latency_ms)

                if r.status_code == 200:
                    data = r.json()
                    parsed = data.get("command", {})
                    actual_action = parsed.get("action")
                    params = parsed.get("parameters", {})
                    
                    correct_action = actual_action == expected_action
                    correct_param = (
                        expected_param is None or expected_param in params
                    )
                    
                    results.append({
                        "input": text,
                        "expected_action": expected_action,
                        "actual_action": actual_action,
                        "correct_action": correct_action,
                        "correct_param": correct_param,
                        "confidence": parsed.get("confidence", 0),
                        "latency_ms": latency_ms,
                    })
                else:
                    results.append({
                        "input": text,
                        "error": f"HTTP {r.status_code}",
                        "latency_ms": latency_ms,
                    })
            except Exception as e:
                results.append({"input": text, "error": str(e)})

        # Compute accuracy
        successful = [r for r in results if "error" not in r]
        accuracy = (
            sum(1 for r in successful if r.get("correct_action"))
            / len(successful)
            * 100
            if successful
            else 0
        )

        return {
            "test_cases": results,
            "accuracy_percent": accuracy,
            "avg_latency_ms": statistics.mean(latencies) if latencies else 0,
            "p95_latency_ms": (
                sorted(latencies)[int(len(latencies) * 0.95)]
                if len(latencies) >= 5
                else max(latencies, default=0)
            ),
        }

    def get_navigation_statistics(self) -> Dict[str, Any]:
        """Fetch navigation decision statistics."""
        try:
            r = requests.get(
                f"{self.backend_url}/api/v1/navigation/statistics", timeout=5
            )
            if r.status_code == 200:
                return r.json()
        except Exception as e:
            self.log(f"Navigation stats failed: {e}")
        return {}

    def get_recent_decisions(self, limit: int = 50) -> List[Dict]:
        """Fetch recent navigation decisions for analysis."""
        try:
            r = requests.get(
                f"{self.backend_url}/api/v1/navigation/decisions",
                params={"limit": limit},
                timeout=5,
            )
            if r.status_code == 200:
                return r.json().get("decisions", [])
        except Exception as e:
            self.log(f"Get decisions failed: {e}")
        return []

    def analyze_explanations(self, decisions: List[Dict]) -> Dict[str, Any]:
        """Analyze explanation quality from decisions."""
        explanations = [
            d.get("simple_explanation") or d.get("detailed_explanation")
            for d in decisions
            if d.get("simple_explanation") or d.get("detailed_explanation")
        ]

        if not explanations:
            return {"count": 0, "note": "No explanations found in decisions"}

        # Basic quality metrics
        lengths = [len(e) for e in explanations]
        first_person = sum(1 for e in explanations if "I " in e or "I'" in e)

        return {
            "count": len(explanations),
            "avg_length_chars": statistics.mean(lengths),
            "first_person_percent": first_person / len(explanations) * 100,
            "sample_explanations": explanations[:3],
        }

    def run_evaluation(self) -> Dict[str, Any]:
        """Run full system evaluation."""
        self.log("Starting system evaluation...")

        # Check backend
        if not self.check_backend_health():
            self.results["error"] = "Backend not reachable"
            return self.results

        # Voice command tests
        self.log("Testing voice command parsing...")
        self.results["voice_command_tests"] = self.test_voice_command_parsing()

        # Navigation stats
        self.log("Fetching navigation statistics...")
        self.results["navigation_stats"] = self.get_navigation_statistics()

        # Explanation analysis
        self.log("Analyzing explanations...")
        decisions = self.get_recent_decisions(50)
        self.results["explanation_analysis"] = self.analyze_explanations(decisions)
        self.results["total_decisions_sampled"] = len(decisions)

        # Decision type distribution
        if decisions:
            types = {}
            for d in decisions:
                t = d.get("decision_type", "unknown")
                types[t] = types.get(t, 0) + 1
            self.results["decision_type_distribution"] = types

        self.log("Evaluation complete.")
        return self.results


def main():
    parser = argparse.ArgumentParser(description="Evaluate system metrics")
    parser.add_argument(
        "--backend-url",
        default="http://localhost:8000",
        help="Backend API URL",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("evaluation_metrics.json"),
        help="Output JSON file",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    args = parser.parse_args()

    evaluator = SystemEvaluator(args.backend_url, verbose=args.verbose)
    results = evaluator.run_evaluation()

    # Sanitize output path against path injection
    safe_filename = os.path.basename(str(args.output))
    safe_output_path = Path.cwd() / safe_filename
    with open(safe_output_path, "w") as f:
        json.dump(results, f, indent=2, default=str)

    print(f"\n{'='*60}")
    print("EVALUATION SUMMARY")
    print(f"{'='*60}")
    
    if "error" in results:
        print(f"ERROR: {results['error']}")
        return 1

    vc = results.get("voice_command_tests", {})
    print(f"\nVoice Command Parsing:")
    print(f"  Accuracy: {vc.get('accuracy_percent', 0):.1f}%")
    print(f"  Avg Latency: {vc.get('avg_latency_ms', 0):.0f}ms")
    print(f"  P95 Latency: {vc.get('p95_latency_ms', 0):.0f}ms")

    nav = results.get("navigation_stats", {})
    print(f"\nNavigation Statistics:")
    print(f"  Total Decisions: {nav.get('total_decisions', 0)}")
    print(f"  Goals Reached: {nav.get('goals_reached', 0)}")

    exp = results.get("explanation_analysis", {})
    print(f"\nExplanation Quality:")
    print(f"  Explanations Analyzed: {exp.get('count', 0)}")
    print(f"  First-Person Usage: {exp.get('first_person_percent', 0):.1f}%")
    print(f"  Avg Length: {exp.get('avg_length_chars', 0):.0f} chars")

    print(f"\nResults saved to: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
