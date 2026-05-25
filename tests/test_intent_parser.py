from __future__ import annotations

import unittest

from ai.intent_parser import IntentParser


class IntentParserTests(unittest.TestCase):
    def test_beg_requests_are_not_supported(self) -> None:
        parser = IntentParser()

        for text in ("beg", "begging", "讨食", "求食"):
            with self.subTest(text=text):
                self.assertEqual(parser.parse(text).name, "unknown")


if __name__ == "__main__":
    unittest.main()
