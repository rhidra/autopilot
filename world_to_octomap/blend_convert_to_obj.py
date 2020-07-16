import sys
import bpy

argv = sys.argv
argv = argv[argv.index("--") + 1:]

# input_file = '/home/rhidra/test.dae'
# output_file = '/home/rhidra/test.obj'

input_file = argv[0]
output_file = argv[1]

print('Input file:', input_file)
print('Output file:', output_file)
print('Converting with blender...')

# Removing default cube, camera and light,
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# Importing and exporting the scene
bpy.ops.wm.collada_import(filepath=input_file)
bpy.ops.export_scene.obj(filepath=output_file, axis_up='Z')

print('-' * 30)
print('  Converting done !')
print('-' * 30)
