//! Requires Vulkan loader + compatible GPU. Run with: `cargo test -- --ignored`

mod common;

use ash::vk;
use ash::vk::Handle;
use dual_spacetime_simulator::shader_blobs;
use gpu_allocator::vulkan::{AllocationCreateDesc, AllocationScheme};
use gpu_allocator::MemoryLocation;
use std::ptr;

#[test]
#[ignore = "requires Vulkan device"]
fn headless_vulkan_initializes() {
    let Some(v) = common::try_create_headless_vulkan() else {
        panic!("Vulkan initialization failed (no loader or no graphics queue)");
    };
    assert!(!v.physical_device.is_null());
    assert!(!v.device.handle().is_null());
    assert!(shader_blobs::TREE_COMPUTE.len() >= 4);
}

#[test]
#[ignore = "requires Vulkan device"]
fn allocator_buffer_write_and_many_allocations() {
    let v = common::try_create_headless_vulkan().expect("vulkan");
    let alloc = v.allocator.as_ref().unwrap();

    let buffer_ci = vk::BufferCreateInfo::default()
        .size(1024)
        .usage(vk::BufferUsageFlags::TRANSFER_SRC)
        .sharing_mode(vk::SharingMode::EXCLUSIVE);
    let buffer = unsafe { v.device.create_buffer(&buffer_ci, None) }.unwrap();
    let requirements = unsafe { v.device.get_buffer_memory_requirements(buffer) };
    let allocation = alloc
        .lock()
        .unwrap()
        .allocate(&AllocationCreateDesc {
            name: "test_buf",
            requirements,
            location: MemoryLocation::CpuToGpu,
            linear: true,
            allocation_scheme: AllocationScheme::GpuAllocatorManaged,
        })
        .unwrap();
    unsafe {
        v.device
            .bind_buffer_memory(buffer, allocation.memory(), allocation.offset())
            .unwrap();
    }
    if let Some(mapped) = allocation.mapped_ptr() {
        let pat = [0xabu8; 1024];
        unsafe {
            ptr::copy_nonoverlapping(pat.as_ptr(), mapped.as_ptr() as *mut u8, 1024);
        }
    }
    unsafe { v.device.destroy_buffer(buffer, None) };
    alloc.lock().unwrap().free(allocation).unwrap();

    for i in 0..100 {
        let buffer_ci = vk::BufferCreateInfo::default()
            .size(256)
            .usage(vk::BufferUsageFlags::STORAGE_BUFFER)
            .sharing_mode(vk::SharingMode::EXCLUSIVE);
        let buffer = unsafe { v.device.create_buffer(&buffer_ci, None) }.unwrap();
        let requirements = unsafe { v.device.get_buffer_memory_requirements(buffer) };
        let allocation = alloc
            .lock()
            .unwrap()
            .allocate(&AllocationCreateDesc {
                name: "stress",
                requirements,
                location: MemoryLocation::CpuToGpu,
                linear: true,
                allocation_scheme: AllocationScheme::GpuAllocatorManaged,
            })
            .unwrap();
        unsafe {
            v.device
                .bind_buffer_memory(buffer, allocation.memory(), allocation.offset())
                .unwrap();
            v.device.destroy_buffer(buffer, None);
        }
        alloc.lock().unwrap().free(allocation).unwrap();
        let _ = i;
    }
}
