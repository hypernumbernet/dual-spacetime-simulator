//! GPU buffer/image allocation helpers built on `gpu-allocator`.

use ash::vk;
use gpu_allocator::vulkan::{Allocation, AllocationCreateDesc, AllocationScheme, Allocator};
use gpu_allocator::MemoryLocation;
use std::sync::Mutex;

pub struct AllocatedBuffer {
    pub buffer: vk::Buffer,
    pub allocation: Option<Allocation>,
}

impl AllocatedBuffer {
    /// Allocates a GPU buffer with VMA allocation and optional mapped memory.
    pub fn new(
        device: &ash::Device,
        allocator: &Mutex<Allocator>,
        size: u64,
        usage: vk::BufferUsageFlags,
        location: MemoryLocation,
        name: &str,
    ) -> Self {
        let buffer_ci = vk::BufferCreateInfo::default()
            .size(size.max(1))
            .usage(usage)
            .sharing_mode(vk::SharingMode::EXCLUSIVE);
        let buffer = unsafe { device.create_buffer(&buffer_ci, None) }.unwrap();
        let requirements = unsafe { device.get_buffer_memory_requirements(buffer) };

        let allocation = allocator
            .lock()
            .unwrap()
            .allocate(&AllocationCreateDesc {
                name,
                requirements,
                location,
                linear: true,
                allocation_scheme: AllocationScheme::GpuAllocatorManaged,
            })
            .unwrap();

        unsafe {
            device
                .bind_buffer_memory(buffer, allocation.memory(), allocation.offset())
                .unwrap();
        }
        Self {
            buffer,
            allocation: Some(allocation),
        }
    }

    /// Destroys buffer and frees associated VMA allocation.
    pub fn destroy(mut self, device: &ash::Device, allocator: &Mutex<Allocator>) {
        unsafe { device.destroy_buffer(self.buffer, None) };
        if let Some(alloc) = self.allocation.take() {
            allocator.lock().unwrap().free(alloc).unwrap();
        }
    }
}

/// Creates a host-visible buffer, uploads typed data, and returns allocated handle + count.
pub fn create_buffer_with_data<T: bytemuck::Pod>(
    device: &ash::Device,
    allocator: &Mutex<Allocator>,
    data: &[T],
    usage: vk::BufferUsageFlags,
    name: &str,
) -> (AllocatedBuffer, u32) {
    let byte_size = (std::mem::size_of::<T>() * data.len().max(1)) as u64;
    let buf = AllocatedBuffer::new(device, allocator, byte_size, usage, MemoryLocation::CpuToGpu, name);

    if !data.is_empty()
        && let Some(ref alloc) = buf.allocation
        && let Some(mapped) = alloc.mapped_ptr()
    {
        let bytes = bytemuck::cast_slice::<T, u8>(data);
        unsafe {
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), mapped.as_ptr() as *mut u8, bytes.len());
        }
    }
    let count = data.len() as u32;
    (buf, count)
}

/// A GPU image plus its allocation and view (e.g. depth buffer or texture atlas).
pub struct AllocatedImage {
    pub image: vk::Image,
    pub allocation: Option<Allocation>,
    pub view: vk::ImageView,
}

impl AllocatedImage {
    /// Creates a 2D image with a device-local allocation and a matching image view.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        device: &ash::Device,
        allocator: &Mutex<Allocator>,
        width: u32,
        height: u32,
        format: vk::Format,
        usage: vk::ImageUsageFlags,
        aspect: vk::ImageAspectFlags,
        name: &str,
    ) -> Self {
        let image_ci = vk::ImageCreateInfo::default()
            .image_type(vk::ImageType::TYPE_2D)
            .format(format)
            .extent(vk::Extent3D {
                width,
                height,
                depth: 1,
            })
            .mip_levels(1)
            .array_layers(1)
            .samples(vk::SampleCountFlags::TYPE_1)
            .tiling(vk::ImageTiling::OPTIMAL)
            .usage(usage)
            .sharing_mode(vk::SharingMode::EXCLUSIVE)
            .initial_layout(vk::ImageLayout::UNDEFINED);
        let image = unsafe { device.create_image(&image_ci, None) }.unwrap();
        let requirements = unsafe { device.get_image_memory_requirements(image) };

        let allocation = allocator
            .lock()
            .unwrap()
            .allocate(&AllocationCreateDesc {
                name,
                requirements,
                location: MemoryLocation::GpuOnly,
                linear: false,
                allocation_scheme: AllocationScheme::GpuAllocatorManaged,
            })
            .unwrap();

        unsafe {
            device
                .bind_image_memory(image, allocation.memory(), allocation.offset())
                .unwrap();
        }

        let view_ci = vk::ImageViewCreateInfo::default()
            .image(image)
            .view_type(vk::ImageViewType::TYPE_2D)
            .format(format)
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask: aspect,
                base_mip_level: 0,
                level_count: 1,
                base_array_layer: 0,
                layer_count: 1,
            });
        let view = unsafe { device.create_image_view(&view_ci, None) }.unwrap();

        Self {
            image,
            allocation: Some(allocation),
            view,
        }
    }

    /// Destroys the view, image, and frees the allocation.
    pub fn destroy(&mut self, device: &ash::Device, allocator: &Mutex<Allocator>) {
        unsafe {
            device.destroy_image_view(self.view, None);
            device.destroy_image(self.image, None);
        }
        if let Some(alloc) = self.allocation.take() {
            allocator.lock().unwrap().free(alloc).unwrap();
        }
    }
}

/// Picks the first supported depth format with optimal-tiling depth-stencil support.
pub fn select_depth_format(instance: &ash::Instance, pd: vk::PhysicalDevice) -> vk::Format {
    for &fmt in &[
        vk::Format::D32_SFLOAT,
        vk::Format::D32_SFLOAT_S8_UINT,
        vk::Format::D24_UNORM_S8_UINT,
    ] {
        let props = unsafe { instance.get_physical_device_format_properties(pd, fmt) };
        if props
            .optimal_tiling_features
            .contains(vk::FormatFeatureFlags::DEPTH_STENCIL_ATTACHMENT)
        {
            return fmt;
        }
    }
    panic!("No supported depth format found");
}

/// Creates a depth image suitable for render-pass depth attachments.
pub fn create_depth_image(
    device: &ash::Device,
    allocator: &Mutex<Allocator>,
    format: vk::Format,
    extent: vk::Extent2D,
    name: &str,
) -> AllocatedImage {
    AllocatedImage::new(
        device,
        allocator,
        extent.width.max(1),
        extent.height.max(1),
        format,
        vk::ImageUsageFlags::DEPTH_STENCIL_ATTACHMENT,
        vk::ImageAspectFlags::DEPTH,
        name,
    )
}
