#!/usr/bin/env python3
import math
import sys

def pol2cart(theta, rho):
    x = rho * math.cos(theta)
    y = rho * math.sin(theta)
    return (x, y)

if __name__ == "__main__":
    args = [float(sys.argv[1]), float(sys.argv[2])]
    ans = pol2cart(args[0], args[1])
    print(ans)
