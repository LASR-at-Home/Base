import torch
import torch.nn.functional as F


def load_torch_model(model, optimizer, path="model.pth", cpu_only=False):
    if cpu_only:
        checkpoint = torch.load(path, map_location=torch.device('cpu'))
    else:
        checkpoint = torch.load(path)
    model.load_state_dict(checkpoint['model_state_dict'])
    if optimizer is not None:
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    epoch = checkpoint['epoch']
    best_val_loss = checkpoint.get('best_val_loss', float('inf'))
    return model, optimizer, epoch, best_val_loss


def binary_erosion_dilation(tensor, thresholds, erosion_iterations=1, dilation_iterations=1):
    """
    Apply binary threshold, followed by erosion and dilation to a tensor.

    :param tensor: Input tensor (N, C, H, W)
    :param thresholds: List of threshold values for each channel
    :param erosion_iterations: Number of erosion iterations
    :param dilation_iterations: Number of dilation iterations
    :return: Processed tensor
    """

    # Check if the length of thresholds matches the number of channels
    if len(thresholds) != tensor.size(1):
        raise ValueError(
            "Length of thresholds must match the number of channels")

    # Binary thresholding
    for i, threshold in enumerate(thresholds):
        tensor[:, i] = (tensor[:, i] > threshold/2).float() / 4
        tensor[:, i] += (tensor[:, i] > threshold).float()
        tensor[:, i] /= max(tensor[:, i].clone())

    # Define the 3x3 kernel for erosion and dilation
    kernel = torch.tensor([[1, 1, 1],
                           [1, 1, 1],
                           [1, 1, 1]], dtype=torch.float32).unsqueeze(0).unsqueeze(0)

    # Replicate the kernel for each channel
    kernel = kernel.repeat(tensor.size(1), 1, 1, 1).to(tensor.device)

    # Erosion
    for _ in range(erosion_iterations):
        # 3x3 convolution with groups
        tensor = F.conv2d(tensor, kernel, padding=1, groups=tensor.size(1))
        tensor = (tensor == 9).float()  # Check if all neighboring pixels are 1

    # Dilation
    for _ in range(dilation_iterations):
        # 3x3 convolution with groups
        tensor_dilated = F.conv2d(
            tensor, kernel, padding=1, groups=tensor.size(1))
        # Combine the original and dilated tensors
        tensor = torch.clamp(tensor + tensor_dilated, 0, 1)

    return tensor


def median_color_float(rgb_image: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    mask = mask.bool()
    median_colors = torch.zeros((rgb_image.size(0), mask.size(
        1), rgb_image.size(1)), device=rgb_image.device)
    for i in range(rgb_image.size(0)):
        for j in range(mask.size(1)):
            for k in range(rgb_image.size(1)):
                valid_pixels = torch.masked_select(rgb_image[i, k], mask[i, j])
                if valid_pixels.numel() > 0:
                    median_value = valid_pixels.median()
                else:
                    median_value = torch.tensor(0.0, device=rgb_image.device)
                median_colors[i, j, k] = median_value
    return median_colors  # / 255.0
