from TargetGenerator import TargetGenerator

tg = TargetGenerator()
tg.set_pattern('trot')

for item in tg.status_graph[tg.pattern]:
    tg.origin_target_pos = tg.target_pos
    if 2 in item:
        while 1:
            pass


