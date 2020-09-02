\ENCODING=ISO-8859-1
\Problem name: Configuration1

Minimize
 obj: 10 alpha(1) + 10 beta(1) + 10 alpha(2) + 10 beta(2) + 30 alpha(3)
      + 30 beta(3) + 30 alpha(4) + 30 beta(4) + 30 alpha(5) + 30 beta(5)
      + 30 alpha(6) + 30 beta(6) + 30 alpha(7) + 30 beta(7) + 30 alpha(8)
      + 30 beta(8) + 30 alpha(9) + 30 beta(9) + 30 alpha(10) + 30 beta(10)
Subject To
 c1:   delta#0#0  = 0
 c2:   delta#0#1 + delta#1#0  = 1
 c3:   delta#0#2 + delta#2#0  = 1
 c4:   delta#0#3 + delta#3#0  = 1
 c5:   delta#0#4 + delta#4#0  = 1
 c6:   delta#0#5 + delta#5#0  = 1
 c7:   delta#0#6 + delta#6#0  = 1
 c8:   delta#0#7 + delta#7#0  = 1
 c9:   delta#0#8 + delta#8#0  = 1
 c10:  delta#0#9 + delta#9#0  = 1
 c11:  delta#0#1 + delta#1#0  = 1
 c12:  delta#1#1  = 0
 c13:  delta#1#2 + delta#2#1  = 1
 c14:  delta#1#3 + delta#3#1  = 1
 c15:  delta#1#4 + delta#4#1  = 1
 c16:  delta#1#5 + delta#5#1  = 1
 c17:  delta#1#6 + delta#6#1  = 1
 c18:  delta#1#7 + delta#7#1  = 1
 c19:  delta#1#8 + delta#8#1  = 1
 c20:  delta#1#9 + delta#9#1  = 1
 c21:  delta#0#2 + delta#2#0  = 1
 c22:  delta#1#2 + delta#2#1  = 1
 c23:  delta#2#2  = 0
 c24:  delta#2#3 + delta#3#2  = 1
 c25:  delta#2#4 + delta#4#2  = 1
 c26:  delta#2#5 + delta#5#2  = 1
 c27:  delta#2#6 + delta#6#2  = 1
 c28:  delta#2#7 + delta#7#2  = 1
 c29:  delta#2#8 + delta#8#2  = 1
 c30:  delta#2#9 + delta#9#2  = 1
 c31:  delta#0#3 + delta#3#0  = 1
 c32:  delta#1#3 + delta#3#1  = 1
 c33:  delta#2#3 + delta#3#2  = 1
 c34:  delta#3#3  = 0
 c35:  delta#3#4 + delta#4#3  = 1
 c36:  delta#3#5 + delta#5#3  = 1
 c37:  delta#3#6 + delta#6#3  = 1
 c38:  delta#3#7 + delta#7#3  = 1
 c39:  delta#3#8 + delta#8#3  = 1
 c40:  delta#3#9 + delta#9#3  = 1
 c41:  delta#0#4 + delta#4#0  = 1
 c42:  delta#1#4 + delta#4#1  = 1
 c43:  delta#2#4 + delta#4#2  = 1
 c44:  delta#3#4 + delta#4#3  = 1
 c45:  delta#4#4  = 0
 c46:  delta#4#5 + delta#5#4  = 1
 c47:  delta#4#6 + delta#6#4  = 1
 c48:  delta#4#7 + delta#7#4  = 1
 c49:  delta#4#8 + delta#8#4  = 1
 c50:  delta#4#9 + delta#9#4  = 1
 c51:  delta#0#5 + delta#5#0  = 1
 c52:  delta#1#5 + delta#5#1  = 1
 c53:  delta#2#5 + delta#5#2  = 1
 c54:  delta#3#5 + delta#5#3  = 1
 c55:  delta#4#5 + delta#5#4  = 1
 c56:  delta#5#5  = 0
 c57:  delta#5#6 + delta#6#5  = 1
 c58:  delta#5#7 + delta#7#5  = 1
 c59:  delta#5#8 + delta#8#5  = 1
 c60:  delta#5#9 + delta#9#5  = 1
 c61:  delta#0#6 + delta#6#0  = 1
 c62:  delta#1#6 + delta#6#1  = 1
 c63:  delta#2#6 + delta#6#2  = 1
 c64:  delta#3#6 + delta#6#3  = 1
 c65:  delta#4#6 + delta#6#4  = 1
 c66:  delta#5#6 + delta#6#5  = 1
 c67:  delta#6#6  = 0
 c68:  delta#6#7 + delta#7#6  = 1
 c69:  delta#6#8 + delta#8#6  = 1
 c70:  delta#6#9 + delta#9#6  = 1
 c71:  delta#0#7 + delta#7#0  = 1
 c72:  delta#1#7 + delta#7#1  = 1
 c73:  delta#2#7 + delta#7#2  = 1
 c74:  delta#3#7 + delta#7#3  = 1
 c75:  delta#4#7 + delta#7#4  = 1
 c76:  delta#5#7 + delta#7#5  = 1
 c77:  delta#6#7 + delta#7#6  = 1
 c78:  delta#7#7  = 0
 c79:  delta#7#8 + delta#8#7  = 1
 c80:  delta#7#9 + delta#9#7  = 1
 c81:  delta#0#8 + delta#8#0  = 1
 c82:  delta#1#8 + delta#8#1  = 1
 c83:  delta#2#8 + delta#8#2  = 1
 c84:  delta#3#8 + delta#8#3  = 1
 c85:  delta#4#8 + delta#8#4  = 1
 c86:  delta#5#8 + delta#8#5  = 1
 c87:  delta#6#8 + delta#8#6  = 1
 c88:  delta#7#8 + delta#8#7  = 1
 c89:  delta#8#8  = 0
 c90:  delta#8#9 + delta#9#8  = 1
 c91:  delta#0#9 + delta#9#0  = 1
 c92:  delta#1#9 + delta#9#1  = 1
 c93:  delta#2#9 + delta#9#2  = 1
 c94:  delta#3#9 + delta#9#3  = 1
 c95:  delta#4#9 + delta#9#4  = 1
 c96:  delta#5#9 + delta#9#5  = 1
 c97:  delta#6#9 + delta#9#6  = 1
 c98:  delta#7#9 + delta#9#7  = 1
 c99:  delta#8#9 + delta#9#8  = 1
 c100: delta#9#9  = 0
 c101: x(1) >= 129
 c102: x(2) >= 195
 c103: x(3) >= 89
 c104: x(4) >= 96
 c105: x(5) >= 110
 c106: x(6) >= 120
 c107: x(7) >= 124
 c108: x(8) >= 126
 c109: x(9) >= 135
 c110: x(10) >= 160
 c111: x(1) <= 559
 c112: x(2) <= 744
 c113: x(3) <= 510
 c114: x(4) <= 521
 c115: x(5) <= 555
 c116: x(6) <= 576
 c117: x(7) <= 577
 c118: x(8) <= 573
 c119: x(9) <= 591
 c120: x(10) <= 657
 c121: alpha(1) + x(1) >= 155
 c122: alpha(2) + x(2) >= 258
 c123: alpha(3) + x(3) >= 98
 c124: alpha(4) + x(4) >= 106
 c125: alpha(5) + x(5) >= 123
 c126: alpha(6) + x(6) >= 135
 c127: alpha(7) + x(7) >= 138
 c128: alpha(8) + x(8) >= 140
 c129: alpha(9) + x(9) >= 150
 c130: alpha(10) + x(10) >= 180
 Cns5: alpha(10) - RgCns5  = 0
 c132: beta(1) - x(1) >= -155
 c133: beta(2) - x(2) >= -258
 c134: beta(3) - x(3) >= -98
 c135: beta(4) - x(4) >= -106
 c136: beta(5) - x(5) >= -123
 c137: beta(6) - x(6) >= -135
 c138: beta(7) - x(7) >= -138
 c139: beta(8) - x(8) >= -140
 c140: beta(9) - x(9) >= -150
 c141: beta(10) - x(10) >= -180
 Cns7: beta(10) - RgCns7  = 0
 c143: 655 delta#1#0 - x(1) + x(2) >= 3
 c144: 655 delta#2#0 - x(1) + x(3) >= 15
 c145: 655 delta#3#0 - x(1) + x(4) >= 15
 c146: 655 delta#4#0 - x(1) + x(5) >= 15
 c147: 655 delta#5#0 - x(1) + x(6) >= 15
 c148: 655 delta#6#0 - x(1) + x(7) >= 15
 c149: 655 delta#7#0 - x(1) + x(8) >= 15
 c150: 655 delta#8#0 - x(1) + x(9) >= 15
 c151: 655 delta#9#0 - x(1) + x(10) >= 15
 c152: 655 delta#0#1 + x(1) - x(2) >= 3
 c153: 655 delta#2#1 - x(2) + x(3) >= 15
 c154: 655 delta#3#1 - x(2) + x(4) >= 15
 c155: 655 delta#4#1 - x(2) + x(5) >= 15
 c156: 655 delta#5#1 - x(2) + x(6) >= 15
 c157: 655 delta#6#1 - x(2) + x(7) >= 15
 c158: 655 delta#7#1 - x(2) + x(8) >= 15
 c159: 655 delta#8#1 - x(2) + x(9) >= 15
 c160: 655 delta#9#1 - x(2) + x(10) >= 15
 c161: 655 delta#0#2 + x(1) - x(3) >= 15
 c162: 655 delta#1#2 + x(2) - x(3) >= 15
 c163: 655 delta#3#2 - x(3) + x(4) >= 8
 c164: 655 delta#4#2 - x(3) + x(5) >= 8
 c165: 655 delta#5#2 - x(3) + x(6) >= 8
 c166: 655 delta#6#2 - x(3) + x(7) >= 8
 c167: 655 delta#7#2 - x(3) + x(8) >= 8
 c168: 655 delta#8#2 - x(3) + x(9) >= 8
 c169: 655 delta#9#2 - x(3) + x(10) >= 8
 c170: 655 delta#0#3 + x(1) - x(4) >= 15
 c171: 655 delta#1#3 + x(2) - x(4) >= 15
 c172: 655 delta#2#3 + x(3) - x(4) >= 8
 c173: 655 delta#4#3 - x(4) + x(5) >= 8
 c174: 655 delta#5#3 - x(4) + x(6) >= 8
 c175: 655 delta#6#3 - x(4) + x(7) >= 8
 c176: 655 delta#7#3 - x(4) + x(8) >= 8
 c177: 655 delta#8#3 - x(4) + x(9) >= 8
 c178: 655 delta#9#3 - x(4) + x(10) >= 8
 c179: 655 delta#0#4 + x(1) - x(5) >= 15
 c180: 655 delta#1#4 + x(2) - x(5) >= 15
 c181: 655 delta#2#4 + x(3) - x(5) >= 8
 c182: 655 delta#3#4 + x(4) - x(5) >= 8
 c183: 655 delta#5#4 - x(5) + x(6) >= 8
 c184: 655 delta#6#4 - x(5) + x(7) >= 8
 c185: 655 delta#7#4 - x(5) + x(8) >= 8
 c186: 655 delta#8#4 - x(5) + x(9) >= 8
 c187: 655 delta#9#4 - x(5) + x(10) >= 8
 c188: 655 delta#0#5 + x(1) - x(6) >= 15
 c189: 655 delta#1#5 + x(2) - x(6) >= 15
 c190: 655 delta#2#5 + x(3) - x(6) >= 8
 c191: 655 delta#3#5 + x(4) - x(6) >= 8
 c192: 655 delta#4#5 + x(5) - x(6) >= 8
 c193: 655 delta#6#5 - x(6) + x(7) >= 8
 c194: 655 delta#7#5 - x(6) + x(8) >= 8
 c195: 655 delta#8#5 - x(6) + x(9) >= 8
 c196: 655 delta#9#5 - x(6) + x(10) >= 8
 c197: 655 delta#0#6 + x(1) - x(7) >= 15
 c198: 655 delta#1#6 + x(2) - x(7) >= 15
 c199: 655 delta#2#6 + x(3) - x(7) >= 8
 c200: 655 delta#3#6 + x(4) - x(7) >= 8
 c201: 655 delta#4#6 + x(5) - x(7) >= 8
 c202: 655 delta#5#6 + x(6) - x(7) >= 8
 c203: 655 delta#7#6 - x(7) + x(8) >= 8
 c204: 655 delta#8#6 - x(7) + x(9) >= 8
 c205: 655 delta#9#6 - x(7) + x(10) >= 8
 c206: 655 delta#0#7 + x(1) - x(8) >= 15
 c207: 655 delta#1#7 + x(2) - x(8) >= 15
 c208: 655 delta#2#7 + x(3) - x(8) >= 8
 c209: 655 delta#3#7 + x(4) - x(8) >= 8
 c210: 655 delta#4#7 + x(5) - x(8) >= 8
 c211: 655 delta#5#7 + x(6) - x(8) >= 8
 c212: 655 delta#6#7 + x(7) - x(8) >= 8
 c213: 655 delta#8#7 - x(8) + x(9) >= 8
 c214: 655 delta#9#7 - x(8) + x(10) >= 8
 c215: 655 delta#0#8 + x(1) - x(9) >= 15
 c216: 655 delta#1#8 + x(2) - x(9) >= 15
 c217: 655 delta#2#8 + x(3) - x(9) >= 8
 c218: 655 delta#3#8 + x(4) - x(9) >= 8
 c219: 655 delta#4#8 + x(5) - x(9) >= 8
 c220: 655 delta#5#8 + x(6) - x(9) >= 8
 c221: 655 delta#6#8 + x(7) - x(9) >= 8
 c222: 655 delta#7#8 + x(8) - x(9) >= 8
 c223: 655 delta#9#8 - x(9) + x(10) >= 8
 c224: 655 delta#0#9 + x(1) - x(10) >= 15
 c225: 655 delta#1#9 + x(2) - x(10) >= 15
 c226: 655 delta#2#9 + x(3) - x(10) >= 8
 c227: 655 delta#3#9 + x(4) - x(10) >= 8
 c228: 655 delta#4#9 + x(5) - x(10) >= 8
 c229: 655 delta#5#9 + x(6) - x(10) >= 8
 c230: 655 delta#6#9 + x(7) - x(10) >= 8
 c231: 655 delta#7#9 + x(8) - x(10) >= 8
 c232: 655 delta#8#9 + x(9) - x(10) >= 8
Bounds
 0 <= delta#0#0 <= 1
 0 <= delta#0#1 <= 1
 0 <= delta#1#0 <= 1
 0 <= delta#0#2 <= 1
 0 <= delta#2#0 <= 1
 0 <= delta#0#3 <= 1
 0 <= delta#3#0 <= 1
 0 <= delta#0#4 <= 1
 0 <= delta#4#0 <= 1
 0 <= delta#0#5 <= 1
 0 <= delta#5#0 <= 1
 0 <= delta#0#6 <= 1
 0 <= delta#6#0 <= 1
 0 <= delta#0#7 <= 1
 0 <= delta#7#0 <= 1
 0 <= delta#0#8 <= 1
 0 <= delta#8#0 <= 1
 0 <= delta#0#9 <= 1
 0 <= delta#9#0 <= 1
 0 <= delta#1#1 <= 1
 0 <= delta#1#2 <= 1
 0 <= delta#2#1 <= 1
 0 <= delta#1#3 <= 1
 0 <= delta#3#1 <= 1
 0 <= delta#1#4 <= 1
 0 <= delta#4#1 <= 1
 0 <= delta#1#5 <= 1
 0 <= delta#5#1 <= 1
 0 <= delta#1#6 <= 1
 0 <= delta#6#1 <= 1
 0 <= delta#1#7 <= 1
 0 <= delta#7#1 <= 1
 0 <= delta#1#8 <= 1
 0 <= delta#8#1 <= 1
 0 <= delta#1#9 <= 1
 0 <= delta#9#1 <= 1
 0 <= delta#2#2 <= 1
 0 <= delta#2#3 <= 1
 0 <= delta#3#2 <= 1
 0 <= delta#2#4 <= 1
 0 <= delta#4#2 <= 1
 0 <= delta#2#5 <= 1
 0 <= delta#5#2 <= 1
 0 <= delta#2#6 <= 1
 0 <= delta#6#2 <= 1
 0 <= delta#2#7 <= 1
 0 <= delta#7#2 <= 1
 0 <= delta#2#8 <= 1
 0 <= delta#8#2 <= 1
 0 <= delta#2#9 <= 1
 0 <= delta#9#2 <= 1
 0 <= delta#3#3 <= 1
 0 <= delta#3#4 <= 1
 0 <= delta#4#3 <= 1
 0 <= delta#3#5 <= 1
 0 <= delta#5#3 <= 1
 0 <= delta#3#6 <= 1
 0 <= delta#6#3 <= 1
 0 <= delta#3#7 <= 1
 0 <= delta#7#3 <= 1
 0 <= delta#3#8 <= 1
 0 <= delta#8#3 <= 1
 0 <= delta#3#9 <= 1
 0 <= delta#9#3 <= 1
 0 <= delta#4#4 <= 1
 0 <= delta#4#5 <= 1
 0 <= delta#5#4 <= 1
 0 <= delta#4#6 <= 1
 0 <= delta#6#4 <= 1
 0 <= delta#4#7 <= 1
 0 <= delta#7#4 <= 1
 0 <= delta#4#8 <= 1
 0 <= delta#8#4 <= 1
 0 <= delta#4#9 <= 1
 0 <= delta#9#4 <= 1
 0 <= delta#5#5 <= 1
 0 <= delta#5#6 <= 1
 0 <= delta#6#5 <= 1
 0 <= delta#5#7 <= 1
 0 <= delta#7#5 <= 1
 0 <= delta#5#8 <= 1
 0 <= delta#8#5 <= 1
 0 <= delta#5#9 <= 1
 0 <= delta#9#5 <= 1
 0 <= delta#6#6 <= 1
 0 <= delta#6#7 <= 1
 0 <= delta#7#6 <= 1
 0 <= delta#6#8 <= 1
 0 <= delta#8#6 <= 1
 0 <= delta#6#9 <= 1
 0 <= delta#9#6 <= 1
 0 <= delta#7#7 <= 1
 0 <= delta#7#8 <= 1
 0 <= delta#8#7 <= 1
 0 <= delta#7#9 <= 1
 0 <= delta#9#7 <= 1
 0 <= delta#8#8 <= 1
 0 <= delta#8#9 <= 1
 0 <= delta#9#8 <= 1
 0 <= delta#9#9 <= 1
 0 <= RgCns5 <= 20
 0 <= RgCns7 <= 477
Binaries
 delta#0#0  delta#0#1  delta#1#0  delta#0#2  delta#2#0  delta#0#3  delta#3#0 
 delta#0#4  delta#4#0  delta#0#5  delta#5#0  delta#0#6  delta#6#0  delta#0#7 
 delta#7#0  delta#0#8  delta#8#0  delta#0#9  delta#9#0  delta#1#1  delta#1#2 
 delta#2#1  delta#1#3  delta#3#1  delta#1#4  delta#4#1  delta#1#5  delta#5#1 
 delta#1#6  delta#6#1  delta#1#7  delta#7#1  delta#1#8  delta#8#1  delta#1#9 
 delta#9#1  delta#2#2  delta#2#3  delta#3#2  delta#2#4  delta#4#2  delta#2#5 
 delta#5#2  delta#2#6  delta#6#2  delta#2#7  delta#7#2  delta#2#8  delta#8#2 
 delta#2#9  delta#9#2  delta#3#3  delta#3#4  delta#4#3  delta#3#5  delta#5#3 
 delta#3#6  delta#6#3  delta#3#7  delta#7#3  delta#3#8  delta#8#3  delta#3#9 
 delta#9#3  delta#4#4  delta#4#5  delta#5#4  delta#4#6  delta#6#4  delta#4#7 
 delta#7#4  delta#4#8  delta#8#4  delta#4#9  delta#9#4  delta#5#5  delta#5#6 
 delta#6#5  delta#5#7  delta#7#5  delta#5#8  delta#8#5  delta#5#9  delta#9#5 
 delta#6#6  delta#6#7  delta#7#6  delta#6#8  delta#8#6  delta#6#9  delta#9#6 
 delta#7#7  delta#7#8  delta#8#7  delta#7#9  delta#9#7  delta#8#8  delta#8#9 
 delta#9#8  delta#9#9 
End
