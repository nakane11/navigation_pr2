#!/usr/bin/env python3

import pykakasi

class KKSClient():

    def __init__(self):
        self.kks = pykakasi.kakasi()
        self.kks.setMode('J', 'a')
        self.kks.setMode('H', 'a')
        self.kks.setMode('K', 'a')
        self.kks.setMode('r', 'Kunrei')
        self.converter = self.kks.getConverter()
                    
    def do(self, text):
        roman_text = self.converter.do(text)
        return roman_text
