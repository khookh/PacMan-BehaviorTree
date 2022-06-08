nodes_list = [(96, 184), (72, 184), (48, 184), (48, 208), (8, 208),
              (8, 232), (96, 232), (72, 208), (24, 184), (8, 184), (8, 160),
              (48, 160), (48, 112), (0, 112), (72, 112), (72, 160), (24, 208),
              (72, 136), (48, 64), (48, 40), (8, 64), (8, 40), (8, 8), (48, 8),
              (96, 8), (96, 40), (72, 40), (72, 64), (96, 64), (96, 88), (72, 88),
              (96, 160), (96, 208)]
for i in range(len(nodes_list)):
    elem = nodes_list[i]
    nodes_list.append((216 - elem[0], elem[1]))

nodes_dic = {0: [1, 31, 33], 1: [0, 2, 7], 2: [3, 11, 1], 3: [2, 16], 4: [16, 5], 5: [4, 6], 6: [5, 17, 39], 7: [1, 17],
             8: [16, 9], 9: [8, 10], 10: [9, 11], 11: [10, 2, 15, 12], 12: [11, 13, 14, 18], 13: [12, 46], 14: [12, 17, 30],
             15: [11, 31, 17], 16: [4, 3, 8], 17: [15, 14, 50], 18: [12, 19, 20], 19: [18, 21, 23, 26], 20: [18, 21],
             21: [20, 19, 22], 22: [21, 23], 23: [22, 19, 24], 24: [23, 25], 25: [24, 26, 58], 26: [25, 19, 27],
             27: [26, 28], 28: [27, 29], 29: [28, 30, 62], 30: [29, 14], 31: [15, 0], 32: [6, 7]}
new_dic = {}
for key in nodes_dic.keys():
    set = nodes_dic[key]
    new_set = []
    for elem in set:
        if elem < 33:
            new_set.append(elem + 33)
        else:
            new_set.append(elem - 33)

    new_dic[key + 33] = new_set
nodes_dic.update(new_dic)

for key in nodes_dic.keys():
    set = nodes_dic[key]
    new_set = []
    node_1 = nodes_list[key]
    for elem in set:
        node_2 = nodes_list[elem]
        new_val = [elem, abs(node_2[0] - node_1[0]) + abs(node_2[1] - node_1[1])]
        if key == 13 and elem == 46 or key == 46 and elem == 13:
            new_val = [elem, 1]
        new_set.append(new_val)
    nodes_dic[key] = new_set
