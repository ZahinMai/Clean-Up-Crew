**Auction | Nearest Task**

| $Layout$ | $T[s]$ | $D_{C1}[m]$ | $D_{C2}[m]$ | $I_{C1}[s]$ | $I_{C2}[s]$ | $C_{C1}$ | $C_{C2}$ |
| --- | --- | --- | --- | --- | --- | --- | --- |
| L1 | 306.240 | 17.62 | 57.86 | 216.64 | 10.34 | 13 | 36 |
| L2 | 117.152 | 12.40 | 20.71 | 53.79 | 4.26 | 4 | 18 |
| L3 | 33.664 | 27.46 | 30.64 | 65.09 | 44.10 | 7 | 15 |

**Auction | Sequential**

| $Layout$ | $T[s]$ | $D_{C1}[m]$ | $D_{C2}[m]$ | $I_{C1}[s]$ | $I_{C2}[s]$ | $C_{C1}$ | $C_{C2}$ |
| --- | --- | --- | --- | --- | --- | --- | --- |
| L1 | 316.768 s | 54.422 | 20.449 | 12.896 | 186.784 | 31 | 4 |
| L2 | 128.640 | 20.602 | 20.794 | 10.912 |  14.400 | 14 | 26 |
| L3 | 277.216 | 32.406 | 13.531 | 2.208 | 101.728 | 18 | 8 |

**Swarm**

| $Layout$ | $T[s]$ | $D_{C1}[m]$ | $D_{C2}[m]$ | $I_{C1}[s]$ | $I_{C2}[s]$ | $C_{C1}$ | $C_{C2}$ |
| --- | --- | --- | --- | --- | --- | --- | --- |
| L1 | 252.384 | 48.665 | 40.195 | 0.032 | 0.192 | 96 | 98 |
| L2 | 211.392 | 40.669 | 32.748 | 0.192 | 0.128 | 78 | 92 |
| L3 | 243.520 | 32.861 | 46.714 | 0.256 | 0.096 | 67 | 108 |

**Baseline**

| $Layout$ | $T[s]$ | $I[s]$ | $D[m]$ | $C$ |
| --- | --- | --- | --- | --- |
| L1 | 502.368 | 0.320 | 96.723 | 202 |
| L2 | 463.104 | 0.320 | 88.921 | 192 |
| L3 | 543.200 | 0.128 | 104.697 | 215 |

```markdown
C1: NEAREST_TASK

- trash layout 1 -
Total Idle Time:   216.640 s
Collision Count:   13
Distance covered:   17.624
Time Elapsed:   306.240 s

- trash layout 2 - 
Total Idle Time:   53.792 s
Collision Count:   4
Distance covered:   12.400
Time Elapsed:   117.152 s

- trash layout 3 - 
Total Idle Time:   65.088 s
Collision Count:   7
Distance covered:   27.4570
Time Elapsed:   33.664 s
```

```markdown
C1: SWARM

- trash layout 1 -
Total Idle Time:   0.032 s
Collision Count:   96
Distance covered:   48.665
Time Elapsed:   172.128 s

- trash layout 2 -
Total Idle Time:   0.192 s
Collision Count:   78
Distance covered:   40.669
Time Elapsed:   211.392 s

- trash layout 3 -
Total Idle Time:   0.256 s
Collision Count:   67
Distance covered:   32.861
Time Elapsed:   172.128 s
```

```markdown
C2: NEAREST_TASK

- trash layout 1 - 
Total Idle Time:   10.336 s
Collision Count:   36
Distance covered:   57.862
Time Elapsed:   306.240 s

- trash layout 2 - 
Total Idle Time:   4.256 s
Collision Count:   18
Distance covered:   20.713
Time Elapsed:   117.152 s

- layout 3 - 
Total Idle Time:   44.096 s
Collision Count:   15
Distance covered:   30.638
Time Elapsed:   33.664 s
```

```markdown
C2: SWARM

- trash layout 1 -
Total Idle Time:   0.192 s
Collision Count:   98
Distance covered:   40.195
Time Elapsed:   211.392 s

- trash layout 2 -
Total Idle Time:   0.128 s
Collision Count:   92
Distance covered:   32.748
Time Elapsed:   172.128 s

- trash layout 3 -
Total Idle Time:   0.096 s
Collision Count:   108
Distance covered:   46.714
Time Elapsed:   243.520 s
```

```markdown
C1: NEAREST_TASK

- trash layout 1 -
Total Idle Time:   216.640 s
Collision Count:   13
Distance covered:  17.624 m
Time El   d: se ap 306.240 s

- trash layout 2 -
Total Idle Time:   53.792 s
Collision Count:   4
Distance covered:  12.400 m
Time El   d: se ap 117.152 s

- trash layout 3 -
Total Idle Time:   65.088 s
Collision Count:   7
Distance covered:  27.457 m
Time El   d: se ap 33.664 s
```

```markdown
C2: NEAREST_TASK

- trash layout 1 -
Total Idle Time:   10.336 s
Collision Count:   36
Distance covered:  57.862 m
Time Elapsed: 306.240 s

- trash layout 2 -
Total Idle Time:   4.256 s
Collision Count:   18
Distance covered:  20.713 m
Time El   d: se ap 117.152 s

- trash layout 3 -
Total Idle Time:   44.096 s
Collision Count:   15
Distance covered:  30.638 m
Time El   d: se ap 33.664 s
```

```markdown
 C2: BASELINE

- trash layout 1 -
Total Idle Time:   0.320 s
Collision Count:   202
Distance covered:   96.723
Time Elapsed:   502.368 s

- trash layout 2 -
Total Idle Time:   0.320 s
Collision Count:   192
Distance covered:   88.921
Time Elapsed:   463.104 s

- trash layout 3 -
Total Idle Time:  0.128 s
Collision Count:   215
Distance covered: 104.697 
Time Elapsed:   543.200 s
```
