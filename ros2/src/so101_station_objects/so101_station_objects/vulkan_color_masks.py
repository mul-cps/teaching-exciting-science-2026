"""Single-dispatch Slang/Vulkan integer masks with bounded reusable buffers."""
from pathlib import Path
import numpy as np


class VulkanColorMasks:
    def __init__(self):
        import slangpy as spy
        self.spy = spy
        # Slang selects an actual Vulkan adapter. Refuse CPU/software adapters.
        self.device = spy.Device(type=spy.DeviceType.vulkan, enable_ray_tracing=False,
                                 enable_cuda_interop=False, enable_debug_layers=False)
        info = self.device.info
        description = str(info)
        if any(word in description.lower() for word in ('llvmpipe', 'lavapipe', 'swiftshader', 'software', 'cpu')):
            self.device.close()
            raise RuntimeError('Vulkan software adapter refused: ' + description)
        self.info = {'name': str(info.adapter_name), 'api': 'vulkan',
                     'slangpy': getattr(spy, '__version__', 'unknown')}
        module = self.device.load_module(str(Path(__file__).with_name('color_masks.slang')))
        self.kernel = self.device.create_compute_kernel(self.device.link_program([module], [module.entry_point('main')]))
        table = np.zeros(512, dtype=np.int32)
        indices = np.arange(1, 256, dtype=np.float64)
        table[1:256] = np.rint((255 << 12) / indices).astype(np.int32)
        table[257:] = np.rint((180 << 12) / (6 * indices)).astype(np.int32)
        self.divisions = self.device.create_buffer(data=table, struct_size=4,
                        usage=spy.BufferUsage.shader_resource)
        self.shape = None

    def __call__(self, image):
        height, width = image.shape[:2]
        count = height * width
        if count < 1 or count > 4096 * 4096:
            raise ValueError('Vulkan RGB image exceeds 4096x4096 pixel bound')
        if self.shape != image.shape:
            # Exactly one frame's buffers are retained. Recreate on shape change.
            self.upload = np.zeros((count * 3 + 3) & ~3, dtype=np.uint8)
            self.rgb = self.device.create_buffer(size=self.upload.nbytes,
                        usage=self.spy.BufferUsage.shader_resource)
            self.output = self.device.create_buffer(element_count=(count + 3) // 4, struct_size=4,
                        usage=self.spy.BufferUsage.unordered_access | self.spy.BufferUsage.shader_resource)
            self.shape = image.shape
        self.upload[:count * 3].reshape(image.shape)[:] = image
        self.rgb.copy_from_numpy(self.upload)
        self.kernel.dispatch(thread_count=[(count + 3) // 4, 1, 1], vars={
            'rgb': self.rgb, 'divisions': self.divisions, 'masks': self.output, 'pixelCount': count})
        packed = self.output.to_numpy().view(np.uint8).reshape(-1)[:count].reshape(height, width)
        return {color: (packed & (1 << bit)) != 0 for bit, color in enumerate(
            ('red', 'green', 'blue', 'yellow'))}
