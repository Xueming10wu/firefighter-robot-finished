#!/usr/bin/env python      
# -*- coding: utf-8 -*-

def main():
    a = 0.03273
    b = 0.19336
    c = -0.00339

    while True:
        x = float(raw_input("请输入x\n"))
        y = float(a + b * x + c * x * x)
        print(y)
        print("\n")

if __name__ == '__main__':
    main()