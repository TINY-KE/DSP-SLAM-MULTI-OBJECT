

object_class_table = {
    "cars": [2],
    "benches": [13], # 板凳
    "backpack": [24], # 背包
    "chairs": [56], # 椅子
    "counchs": [57], # 沙发
    "bottles": [39], # 瓶子
    "wine_glasses": [40], # 酒杯
    "cups": [41], # 杯子
    "bowls": [45], # 碗
    "bananas": [46], "apples": [47], "oranges": [49],
    "potted_plants": [58], # 盆栽植物
    "beds": [59], #床
    "dining_tables": [60], #桌子
    "tv_monitor": [62],
    "laptop": [63],
    "mouse": [64],
    "keyboard": [66],
    "microwave": [68], "oven":[69], "toaster": [70], "refrigerator": [72],
    "book": [73], "clock": [74], "vase": [75], "teddy_bears": [77]
}


object_classes = list(object_class_table.keys())

# object_classes_on_ground = ["benches", "chairs", "potted_plants", "beds", "dining_tables", "refrigerator"]
object_classes_on_ground = ["cars","benches", "chairs", "counchs","beds", "dining_tables", "refrigerator"]
object_classes_on_ground = [c for c in object_classes]
# object_classes_on_ground = ["cars", "chairs", "counchs","beds", "dining_tables", "refrigerator"]
# object_classes_on_ground = [c for c in object_classes if c != "beds"]
# object_classes_on_ground = [c for c in object_classes if c == "laptop"]
# object_classes_on_ground = [c for c in object_classes if c == "dining_tables"]


object_classes_on_table = [c for c in object_classes if c not in object_classes_on_ground ]