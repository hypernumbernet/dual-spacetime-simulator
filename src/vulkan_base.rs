use ash::khr::{surface, swapchain};
use ash::{vk, Device, Entry, Instance};
use gpu_allocator::vulkan::{Allocator, AllocatorCreateDesc};
use raw_window_handle::{HasDisplayHandle, HasWindowHandle};
use std::sync::{Arc, Mutex};
use winit::window::Window;

const MAX_FRAMES_IN_FLIGHT: usize = 2;

pub struct VulkanBase {
    #[allow(dead_code)]
    pub entry: Entry,
    pub instance: Instance,
    pub surface_loader: surface::Instance,
    pub surface: vk::SurfaceKHR,
    pub physical_device: vk::PhysicalDevice,
    pub device: Device,
    pub graphics_queue: vk::Queue,
    pub graphics_queue_family: u32,
    pub swapchain_loader: swapchain::Device,
    pub swapchain: vk::SwapchainKHR,
    pub swapchain_images: Vec<vk::Image>,
    pub swapchain_image_views: Vec<vk::ImageView>,
    pub swapchain_format: vk::Format,
    pub swapchain_extent: vk::Extent2D,
    pub command_pool: vk::CommandPool,
    pub command_buffers: Vec<vk::CommandBuffer>,
    pub image_available_semaphores: Vec<vk::Semaphore>,
    pub render_finished_semaphores: Vec<vk::Semaphore>,
    pub in_flight_fences: Vec<vk::Fence>,
    pub current_frame: usize,
    pub allocator: Option<Arc<Mutex<Allocator>>>,
}

impl VulkanBase {
    pub fn new(window: &Window) -> Self {
        let entry = unsafe { Entry::load() }.expect("Failed to load Vulkan");

        let display_handle = window.display_handle().unwrap().as_raw();
        let required_extensions = ash_window::enumerate_required_extensions(display_handle)
            .expect("Failed to enumerate required extensions");

        let app_info = vk::ApplicationInfo::default()
            .application_name(c"DualSpacetimeSimulator")
            .application_version(vk::make_api_version(0, 0, 2, 0))
            .engine_name(c"No Engine")
            .engine_version(vk::make_api_version(0, 1, 0, 0))
            .api_version(vk::API_VERSION_1_2);

        let instance_ci = vk::InstanceCreateInfo::default()
            .application_info(&app_info)
            .enabled_extension_names(required_extensions);

        let instance = unsafe { entry.create_instance(&instance_ci, None) }
            .expect("Failed to create Vulkan instance");

        let surface_loader = surface::Instance::new(&entry, &instance);
        let window_handle = window.window_handle().unwrap().as_raw();
        let surface = unsafe {
            ash_window::create_surface(&entry, &instance, display_handle, window_handle, None)
        }
        .expect("Failed to create surface");

        let (physical_device, queue_family) =
            pick_physical_device(&instance, &surface_loader, surface);

        let queue_priorities = [1.0f32];
        let queue_ci = vk::DeviceQueueCreateInfo::default()
            .queue_family_index(queue_family)
            .queue_priorities(&queue_priorities);
        let queue_cis = [queue_ci];

        let device_extensions = [swapchain::NAME.as_ptr()];
        let device_ci = vk::DeviceCreateInfo::default()
            .queue_create_infos(&queue_cis)
            .enabled_extension_names(&device_extensions);

        let device = unsafe { instance.create_device(physical_device, &device_ci, None) }
            .expect("Failed to create logical device");
        let graphics_queue = unsafe { device.get_device_queue(queue_family, 0) };

        let swapchain_loader = swapchain::Device::new(&instance, &device);

        let (swapchain, images, format, extent) = create_swapchain(
            &surface_loader,
            &swapchain_loader,
            physical_device,
            surface,
            window,
            vk::SwapchainKHR::null(),
        );
        let image_views = create_image_views(&device, &images, format);

        let pool_ci = vk::CommandPoolCreateInfo::default()
            .queue_family_index(queue_family)
            .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER);
        let command_pool = unsafe { device.create_command_pool(&pool_ci, None) }.unwrap();

        let alloc_ci = vk::CommandBufferAllocateInfo::default()
            .command_pool(command_pool)
            .level(vk::CommandBufferLevel::PRIMARY)
            .command_buffer_count(MAX_FRAMES_IN_FLIGHT as u32);
        let command_buffers = unsafe { device.allocate_command_buffers(&alloc_ci) }.unwrap();

        let (image_available, render_finished, in_flight) = create_sync_objects(&device);

        let allocator = Allocator::new(&AllocatorCreateDesc {
            instance: instance.clone(),
            device: device.clone(),
            physical_device,
            debug_settings: Default::default(),
            buffer_device_address: false,
            allocation_sizes: Default::default(),
        })
        .expect("Failed to create GPU allocator");

        Self {
            entry,
            instance,
            surface_loader,
            surface,
            physical_device,
            device,
            graphics_queue,
            graphics_queue_family: queue_family,
            swapchain_loader,
            swapchain,
            swapchain_images: images,
            swapchain_image_views: image_views,
            swapchain_format: format,
            swapchain_extent: extent,
            command_pool,
            command_buffers,
            image_available_semaphores: image_available,
            render_finished_semaphores: render_finished,
            in_flight_fences: in_flight,
            current_frame: 0,
            allocator: Some(Arc::new(Mutex::new(allocator))),
        }
    }

    pub fn recreate_swapchain(&mut self, window: &Window) {
        unsafe { self.device.device_wait_idle().unwrap() };

        self.cleanup_swapchain();

        let (sc, images, format, extent) = create_swapchain(
            &self.surface_loader,
            &self.swapchain_loader,
            self.physical_device,
            self.surface,
            window,
            self.swapchain,
        );
        let old = self.swapchain;
        self.swapchain = sc;
        self.swapchain_images = images;
        self.swapchain_format = format;
        self.swapchain_extent = extent;
        self.swapchain_image_views = create_image_views(&self.device, &self.swapchain_images, format);

        if old != vk::SwapchainKHR::null() {
            unsafe { self.swapchain_loader.destroy_swapchain(old, None) };
        }
    }

    fn cleanup_swapchain(&mut self) {
        for &iv in &self.swapchain_image_views {
            unsafe { self.device.destroy_image_view(iv, None) };
        }
        self.swapchain_image_views.clear();
    }

    pub fn wait_for_fence(&self) {
        let fence = self.in_flight_fences[self.current_frame];
        unsafe { self.device.wait_for_fences(&[fence], true, u64::MAX).unwrap() };
    }

    pub fn reset_fence(&self) {
        let fence = self.in_flight_fences[self.current_frame];
        unsafe { self.device.reset_fences(&[fence]).unwrap() };
    }

    pub fn acquire_next_image(&self) -> Result<(u32, bool), vk::Result> {
        let semaphore = self.image_available_semaphores[self.current_frame];
        unsafe {
            self.swapchain_loader
                .acquire_next_image(self.swapchain, u64::MAX, semaphore, vk::Fence::null())
        }
    }

    pub fn current_command_buffer(&self) -> vk::CommandBuffer {
        self.command_buffers[self.current_frame]
    }

    pub fn submit_and_present(&self, image_index: u32) -> Result<bool, vk::Result> {
        let wait_semaphores = [self.image_available_semaphores[self.current_frame]];
        let wait_stages = [vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT];
        let signal_semaphores = [self.render_finished_semaphores[self.current_frame]];
        let command_buffers = [self.command_buffers[self.current_frame]];

        let submit_info = vk::SubmitInfo::default()
            .wait_semaphores(&wait_semaphores)
            .wait_dst_stage_mask(&wait_stages)
            .command_buffers(&command_buffers)
            .signal_semaphores(&signal_semaphores);

        let fence = self.in_flight_fences[self.current_frame];
        unsafe {
            self.device
                .queue_submit(self.graphics_queue, &[submit_info], fence)
                .expect("Failed to submit draw command buffer");
        }

        let swapchains = [self.swapchain];
        let image_indices = [image_index];
        let present_info = vk::PresentInfoKHR::default()
            .wait_semaphores(&signal_semaphores)
            .swapchains(&swapchains)
            .image_indices(&image_indices);

        unsafe { self.swapchain_loader.queue_present(self.graphics_queue, &present_info) }
    }

    pub fn advance_frame(&mut self) {
        self.current_frame = (self.current_frame + 1) % MAX_FRAMES_IN_FLIGHT;
    }

}

impl Drop for VulkanBase {
    fn drop(&mut self) {
        unsafe {
            self.device.device_wait_idle().unwrap();

            for &sem in &self.image_available_semaphores {
                self.device.destroy_semaphore(sem, None);
            }
            for &sem in &self.render_finished_semaphores {
                self.device.destroy_semaphore(sem, None);
            }
            for &fence in &self.in_flight_fences {
                self.device.destroy_fence(fence, None);
            }

            self.device.destroy_command_pool(self.command_pool, None);

            // Drop allocator before device since it may reference device internals
            drop(self.allocator.take());

            self.cleanup_swapchain();
            self.swapchain_loader
                .destroy_swapchain(self.swapchain, None);
            self.surface_loader.destroy_surface(self.surface, None);
            self.device.destroy_device(None);
            self.instance.destroy_instance(None);
        }
    }
}

fn pick_physical_device(
    instance: &Instance,
    surface_loader: &surface::Instance,
    surface: vk::SurfaceKHR,
) -> (vk::PhysicalDevice, u32) {
    let devices = unsafe { instance.enumerate_physical_devices() }.unwrap();
    for pd in &devices {
        let queue_families =
            unsafe { instance.get_physical_device_queue_family_properties(*pd) };
        for (i, qf) in queue_families.iter().enumerate() {
            let supports_graphics = qf.queue_flags.contains(vk::QueueFlags::GRAPHICS);
            let supports_surface = unsafe {
                surface_loader.get_physical_device_surface_support(*pd, i as u32, surface)
            }
            .unwrap_or(false);
            if supports_graphics && supports_surface {
                return (*pd, i as u32);
            }
        }
    }
    panic!("No suitable physical device found");
}

fn create_swapchain(
    surface_loader: &surface::Instance,
    swapchain_loader: &swapchain::Device,
    physical_device: vk::PhysicalDevice,
    surface: vk::SurfaceKHR,
    window: &Window,
    old_swapchain: vk::SwapchainKHR,
) -> (vk::SwapchainKHR, Vec<vk::Image>, vk::Format, vk::Extent2D) {
    let caps = unsafe {
        surface_loader.get_physical_device_surface_capabilities(physical_device, surface)
    }
    .unwrap();
    let formats = unsafe {
        surface_loader.get_physical_device_surface_formats(physical_device, surface)
    }
    .unwrap();
    let present_modes = unsafe {
        surface_loader.get_physical_device_surface_present_modes(physical_device, surface)
    }
    .unwrap();

    let surface_format = formats
        .iter()
        .find(|f| {
            f.format == vk::Format::B8G8R8A8_UNORM
                && f.color_space == vk::ColorSpaceKHR::SRGB_NONLINEAR
        })
        .unwrap_or(&formats[0]);

    let present_mode = if present_modes.contains(&vk::PresentModeKHR::MAILBOX) {
        vk::PresentModeKHR::MAILBOX
    } else {
        vk::PresentModeKHR::FIFO
    };

    let window_size = window.inner_size();
    let extent = if caps.current_extent.width != u32::MAX {
        caps.current_extent
    } else {
        vk::Extent2D {
            width: window_size
                .width
                .clamp(caps.min_image_extent.width, caps.max_image_extent.width),
            height: window_size
                .height
                .clamp(caps.min_image_extent.height, caps.max_image_extent.height),
        }
    };

    let image_count = (caps.min_image_count + 1).min(if caps.max_image_count > 0 {
        caps.max_image_count
    } else {
        u32::MAX
    });

    let ci = vk::SwapchainCreateInfoKHR::default()
        .surface(surface)
        .min_image_count(image_count)
        .image_format(surface_format.format)
        .image_color_space(surface_format.color_space)
        .image_extent(extent)
        .image_array_layers(1)
        .image_usage(vk::ImageUsageFlags::COLOR_ATTACHMENT)
        .image_sharing_mode(vk::SharingMode::EXCLUSIVE)
        .pre_transform(caps.current_transform)
        .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
        .present_mode(present_mode)
        .clipped(true)
        .old_swapchain(old_swapchain);

    let swapchain = unsafe { swapchain_loader.create_swapchain(&ci, None) }.unwrap();
    let images = unsafe { swapchain_loader.get_swapchain_images(swapchain) }.unwrap();

    (swapchain, images, surface_format.format, extent)
}

fn create_image_views(
    device: &Device,
    images: &[vk::Image],
    format: vk::Format,
) -> Vec<vk::ImageView> {
    images
        .iter()
        .map(|&image| {
            let ci = vk::ImageViewCreateInfo::default()
                .image(image)
                .view_type(vk::ImageViewType::TYPE_2D)
                .format(format)
                .subresource_range(vk::ImageSubresourceRange {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    base_mip_level: 0,
                    level_count: 1,
                    base_array_layer: 0,
                    layer_count: 1,
                });
            unsafe { device.create_image_view(&ci, None) }.unwrap()
        })
        .collect()
}

fn create_sync_objects(
    device: &Device,
) -> (Vec<vk::Semaphore>, Vec<vk::Semaphore>, Vec<vk::Fence>) {
    let sem_ci = vk::SemaphoreCreateInfo::default();
    let fence_ci = vk::FenceCreateInfo::default().flags(vk::FenceCreateFlags::SIGNALED);

    let mut image_available = Vec::with_capacity(MAX_FRAMES_IN_FLIGHT);
    let mut render_finished = Vec::with_capacity(MAX_FRAMES_IN_FLIGHT);
    let mut in_flight = Vec::with_capacity(MAX_FRAMES_IN_FLIGHT);

    for _ in 0..MAX_FRAMES_IN_FLIGHT {
        image_available.push(unsafe { device.create_semaphore(&sem_ci, None) }.unwrap());
        render_finished.push(unsafe { device.create_semaphore(&sem_ci, None) }.unwrap());
        in_flight.push(unsafe { device.create_fence(&fence_ci, None) }.unwrap());
    }

    (image_available, render_finished, in_flight)
}
