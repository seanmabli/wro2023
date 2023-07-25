import random

colors = ["blue", "green"]
pickupColors = ["blue", "blue", "green", "green"]
print(f"marking {colors[random.randint(0, len(colors) - 1)]}, {colors[random.randint(0, len(colors) - 1)]}")
print(f"pickup: {pickupColors.pop(random.randint(0, len(pickupColors) - 1))}, {pickupColors.pop(random.randint(0, len(pickupColors) - 1))},{pickupColors.pop(random.randint(0, len(pickupColors) - 1))}, {pickupColors.pop(random.randint(0, len(pickupColors) - 1))}")