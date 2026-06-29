//! Headless Vulkan helpers for integration tests (no window / surface).

use ash::{Device, Entry, Instance, vk};
use gpu_allocator::vulkan::{Allocator, AllocatorCreateDesc};
use std::sync::{Arc, Mutex};

#[allow(dead_code)]
pub struct HeadlessVulkan {
    #[allow(dead_code)]
    pub entry: Entry,
    pub instance: Instance,
    pub physical_device: vk::PhysicalDevice,
    pub device: Device,
    pub graphics_queue: vk::Queue,
    pub graphics_queue_family: u32,
    pub command_pool: vk::CommandPool,
    pub allocator: Option<Arc<Mutex<Allocator>>>,
}

impl Drop for HeadlessVulkan {
    fn drop(&mut self) {
        unsafe {
            let _ = self.device.device_wait_idle();
            drop(self.allocator.take());
            self.device.destroy_command_pool(self.command_pool, None);
            self.device.destroy_device(None);
            self.instance.destroy_instance(None);
        }
    }
}

fn pick_graphics_queue_family(instance: &Instance, pd: vk::PhysicalDevice) -> Option<u32> {
    let props = unsafe { instance.get_physical_device_queue_family_properties(pd) };
    for (i, qf) in props.iter().enumerate() {
        if qf.queue_flags.contains(vk::QueueFlags::GRAPHICS) {
            return Some(i as u32);
        }
    }
    None
}

/// Returns `None` if Vulkan is unavailable or initialization fails (e.g. CI without GPU).
pub fn try_create_headless_vulkan() -> Option<HeadlessVulkan> {
    let entry = unsafe { Entry::load() }.ok()?;

    let app_info = vk::ApplicationInfo::default()
        .application_name(c"DualSpacetimeSimulatorTests")
        .application_version(vk::make_api_version(0, 0, 2, 0))
        .api_version(vk::API_VERSION_1_2);

    let instance_ci = vk::InstanceCreateInfo::default().application_info(&app_info);
    let instance = unsafe { entry.create_instance(&instance_ci, None) }.ok()?;

    let physical_devices = unsafe { instance.enumerate_physical_devices() }.ok()?;
    let mut chosen_pd = None;
    let mut chosen_family = None;
    for &pd in &physical_devices {
        if let Some(fam) = pick_graphics_queue_family(&instance, pd) {
            chosen_pd = Some(pd);
            chosen_family = Some(fam);
            break;
        }
    }
    let (physical_device, graphics_queue_family) = match (chosen_pd, chosen_family) {
        (Some(pd), Some(f)) => (pd, f),
        _ => {
            unsafe { instance.destroy_instance(None) };
            return None;
        }
    };

    let queue_priorities = [1.0f32];
    let queue_ci = vk::DeviceQueueCreateInfo::default()
        .queue_family_index(graphics_queue_family)
        .queue_priorities(&queue_priorities);
    let queue_cis = [queue_ci];
    let device_ci = vk::DeviceCreateInfo::default().queue_create_infos(&queue_cis);
    let device = unsafe { instance.create_device(physical_device, &device_ci, None) }.ok()?;
    let graphics_queue = unsafe { device.get_device_queue(graphics_queue_family, 0) };

    let pool_ci = vk::CommandPoolCreateInfo::default()
        .queue_family_index(graphics_queue_family)
        .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER);
    let command_pool = unsafe { device.create_command_pool(&pool_ci, None) }.ok()?;

    let allocator = Allocator::new(&AllocatorCreateDesc {
        instance: instance.clone(),
        device: device.clone(),
        physical_device,
        debug_settings: Default::default(),
        buffer_device_address: false,
        allocation_sizes: Default::default(),
    })
    .ok()?;

    Some(HeadlessVulkan {
        entry,
        instance,
        physical_device,
        device,
        graphics_queue,
        graphics_queue_family,
        command_pool,
        allocator: Some(Arc::new(Mutex::new(allocator))),
    })
}

/// Records and submits a one-shot primary command buffer on the graphics queue.
pub fn submit_graphics<F>(v: &HeadlessVulkan, record: F)
where
    F: FnOnce(vk::CommandBuffer),
{
    let alloc_info = vk::CommandBufferAllocateInfo::default()
        .command_pool(v.command_pool)
        .level(vk::CommandBufferLevel::PRIMARY)
        .command_buffer_count(1);
    let cmd_bufs = unsafe { v.device.allocate_command_buffers(&alloc_info) }.unwrap();
    let cmd = cmd_bufs[0];
    let begin_info = vk::CommandBufferBeginInfo::default()
        .flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
    unsafe {
        v.device.begin_command_buffer(cmd, &begin_info).unwrap();
        record(cmd);
        v.device.end_command_buffer(cmd).unwrap();
        let submit_info = vk::SubmitInfo::default().command_buffers(&cmd_bufs);
        v.device
            .queue_submit(v.graphics_queue, &[submit_info], vk::Fence::null())
            .unwrap();
        v.device.queue_wait_idle(v.graphics_queue).unwrap();
        v.device.free_command_buffers(v.command_pool, &cmd_bufs);
    }
}
