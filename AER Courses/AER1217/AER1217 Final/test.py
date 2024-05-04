from itertools import product, permutations

# Define your list of lists
lists = [[1, 2, 3], [4, 5], [6, 7]]
print(lists[0:2])
# Generate all permutations of the elements in the sublists
permutations_ = list(product(*[permutations(sublist) for sublist in lists]))

# Output a list of tuples with all elements in each tuple
output = [tuple(item for sublist in combination for item in sublist) for combination in permutations_]

# Print the output
for item in output:
    print(item)