import math
from math import ceil
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


def _conv1d(in_channel, out_channel, kernel_size, padding):
    return nn.Sequential(
        nn.Conv1d(in_channel, out_channel, kernel_size=kernel_size, padding=padding),
        nn.BatchNorm1d(out_channel),
        nn.LeakyReLU(negative_slope=0.1, inplace=True),
    )


def _conv1d_3(in_channel, out_channel):
    return _conv1d(in_channel, out_channel, kernel_size=3, padding=1)


def rphi_to_xy(r, phi):
    return r * np.cos(phi), r * np.sin(phi)


def canonical_to_global(scan_r, scan_phi, dx, dy):
    tmp_y = scan_r + dy
    tmp_phi = np.arctan2(dx, tmp_y)
    dets_phi = tmp_phi + scan_phi
    dets_r = tmp_y / np.cos(tmp_phi)
    return dets_r, dets_phi


def scans_to_cutout(
    scans,
    scan_phi,
    stride=1,
    centered=True,
    fixed=False,
    window_width=1.66,
    window_depth=1.0,
    num_cutout_pts=48,
    padding_val=29.99,
    area_mode=False,
):
    num_scans, num_pts = scans.shape

    dists = (
        scans[:, ::stride]
        if fixed
        else np.tile(scans[-1, ::stride], num_scans).reshape(num_scans, -1)
    )
    half_alpha = np.arctan(0.5 * window_width / np.maximum(dists, 1e-2))

    delta_alpha = 2.0 * half_alpha / (num_cutout_pts - 1)
    ang_ct = (
        scan_phi[::stride]
        - half_alpha
        + np.arange(num_cutout_pts).reshape(num_cutout_pts, 1, 1) * delta_alpha
    )
    ang_ct = (ang_ct + np.pi) % (2.0 * np.pi) - np.pi
    inds_ct = (ang_ct - scan_phi[0]) / (scan_phi[1] - scan_phi[0])
    outbound_mask = np.logical_or(inds_ct < 0, inds_ct > num_pts - 1)

    inds_ct_low = np.clip(np.floor(inds_ct), 0, num_pts - 1).astype(np.int64)
    inds_ct_high = np.clip(inds_ct_low + 1, 0, num_pts - 1).astype(np.int64)
    inds_ct_ratio = np.clip(inds_ct - inds_ct_low, 0.0, 1.0)
    inds_offset = np.arange(num_scans).reshape(1, num_scans, 1) * num_pts
    ct_low = np.take(scans, inds_ct_low + inds_offset)
    ct_high = np.take(scans, inds_ct_high + inds_offset)
    ct = ct_low + inds_ct_ratio * (ct_high - ct_low)

    if area_mode:
        num_pts_in_window = inds_ct[-1] - inds_ct[0]
        area_mask = num_pts_in_window > num_cutout_pts
        if np.sum(area_mask) > 0:
            s_area = int(math.ceil(np.max(num_pts_in_window) / num_cutout_pts))
            num_ct_pts_area = s_area * num_cutout_pts
            delta_alpha_area = 2.0 * half_alpha / (num_ct_pts_area - 1)
            ang_ct_area = (
                scan_phi[::stride]
                - half_alpha
                + np.arange(num_ct_pts_area).reshape(num_ct_pts_area, 1, 1)
                * delta_alpha_area
            )
            ang_ct_area = (ang_ct_area + np.pi) % (2.0 * np.pi) - np.pi
            inds_ct_area = (ang_ct_area - scan_phi[0]) / (scan_phi[1] - scan_phi[0])
            inds_ct_area = np.rint(np.clip(inds_ct_area, 0, num_pts - 1)).astype(np.int32)
            ct_area = np.take(scans, inds_ct_area + inds_offset)
            ct_area = ct_area.reshape(
                num_cutout_pts, s_area, num_scans, dists.shape[1]
            ).mean(axis=1)
            ct[:, area_mask] = ct_area[:, area_mask]

    ct[outbound_mask] = padding_val
    ct = np.clip(ct, dists - window_depth, dists + window_depth)
    if centered:
        ct = ct - dists
        ct = ct / window_depth

    return np.ascontiguousarray(ct.transpose((2, 1, 0)), dtype=np.float32)


def nms_predicted_center(scan_grid, phi_grid, pred_cls, pred_reg, min_dist=0.5):
    assert len(pred_cls.shape) == 1

    pred_r, pred_phi = canonical_to_global(
        scan_grid, phi_grid, pred_reg[:, 0], pred_reg[:, 1]
    )
    pred_xs, pred_ys = rphi_to_xy(pred_r, pred_phi)

    sort_inds = np.argsort(pred_cls)[::-1]
    pred_xs, pred_ys = pred_xs[sort_inds], pred_ys[sort_inds]
    pred_cls = pred_cls[sort_inds]

    num_pts = len(scan_grid)
    xdiff = pred_xs.reshape(num_pts, 1) - pred_xs.reshape(1, num_pts)
    ydiff = pred_ys.reshape(num_pts, 1) - pred_ys.reshape(1, num_pts)
    p_dist = np.sqrt(np.square(xdiff) + np.square(ydiff))

    keep = np.ones(num_pts, dtype=np.bool_)
    instance_mask = np.zeros(num_pts, dtype=np.int32)
    instance_id = 1
    for i in range(num_pts):
        if not keep[i]:
            continue
        dup_inds = p_dist[i] < min_dist
        keep[dup_inds] = False
        keep[i] = True
        instance_mask[sort_inds[dup_inds]] = instance_id
        instance_id += 1

    det_xys = np.stack((pred_xs, pred_ys), axis=1)[keep]
    det_cls = pred_cls[keep]
    return det_xys, det_cls, instance_mask


class DrowNet(nn.Module):
    def __init__(self, dropout=0.5, cls_loss=None, mixup_alpha=0.0, mixup_w=0.0):
        super(DrowNet, self).__init__()
        self.dropout = dropout
        self.conv_block_1 = nn.Sequential(
            _conv1d_3(1, 64), _conv1d_3(64, 64), _conv1d_3(64, 128)
        )
        self.conv_block_2 = nn.Sequential(
            _conv1d_3(128, 128), _conv1d_3(128, 128), _conv1d_3(128, 256)
        )
        self.conv_block_3 = nn.Sequential(
            _conv1d_3(256, 256), _conv1d_3(256, 256), _conv1d_3(256, 512)
        )
        self.conv_block_4 = nn.Sequential(_conv1d_3(512, 256), _conv1d_3(256, 128))
        self.conv_cls = nn.Conv1d(128, 1, kernel_size=1)
        self.conv_reg = nn.Conv1d(128, 2, kernel_size=1)

    def forward(self, x):
        n_batch, n_cutout, n_scan, n_pts = x.shape
        out = x.view(n_batch * n_cutout * n_scan, 1, n_pts)
        out = self._conv_and_pool(out, self.conv_block_1)
        out = self._conv_and_pool(out, self.conv_block_2)
        out = out.view(n_batch, n_cutout, n_scan, out.shape[-2], out.shape[-1])
        out = torch.sum(out, dim=2)
        out = out.view(n_batch * n_cutout, out.shape[-2], out.shape[-1])
        out = self._conv_and_pool(out, self.conv_block_3)
        out = self.conv_block_4(out)
        out = F.avg_pool1d(out, kernel_size=out.shape[-1])
        pred_cls = self.conv_cls(out).view(n_batch, n_cutout, -1)
        pred_reg = self.conv_reg(out).view(n_batch, n_cutout, 2)
        return pred_cls, pred_reg

    def _conv_and_pool(self, x, conv_block):
        out = conv_block(x)
        out = F.max_pool1d(out, kernel_size=2)
        if self.dropout > 0:
            out = F.dropout(out, p=self.dropout, training=self.training)
        return out


class _SpatialAttentionMemory(nn.Module):
    def __init__(
        self, n_pts, n_channel, embedding_length, alpha, window_size, panoramic_scan
    ):
        super(_SpatialAttentionMemory, self).__init__()
        self._alpha = alpha
        self._window_size = window_size
        self._embedding_length = embedding_length
        self._panoramic_scan = panoramic_scan

        self.conv = nn.Sequential(
            nn.Conv1d(n_channel, self._embedding_length, kernel_size=n_pts, padding=0),
            nn.BatchNorm1d(self._embedding_length),
            nn.LeakyReLU(negative_slope=0.1, inplace=True),
        )
        self._memory = None
        self.neighbor_masks, self.neighbor_inds = None, None

    def reset(self):
        self._memory = None

    def forward(self, x_new):
        if self._memory is None:
            self._memory = x_new
            return self._memory, None

        n_batch, n_cutout, n_channel, n_pts = x_new.shape
        if (
            self.neighbor_masks is None
            or self.neighbor_masks.shape[0] != x_new.shape[1]
        ):
            self.neighbor_masks, self.neighbor_inds = self._generate_neighbor_mask(x_new)

        emb_x = self.conv(x_new.view(n_batch * n_cutout, n_channel, n_pts))
        emb_x = emb_x.view(n_batch, n_cutout, self._embedding_length)

        emb_temp = self.conv(self._memory.view(n_batch * n_cutout, n_channel, n_pts))
        emb_temp = emb_temp.view(n_batch, n_cutout, self._embedding_length)

        sim = torch.matmul(emb_x, emb_temp.permute(0, 2, 1))
        sim = sim - 1e10 * (1.0 - self.neighbor_masks)
        maxes = sim.max(dim=-1, keepdim=True)[0]
        exps = torch.exp(sim - maxes) * self.neighbor_masks
        exps_sum = exps.sum(dim=-1, keepdim=True)
        sim = exps / exps_sum

        atten_memory = self._memory.view(n_batch, n_cutout, n_channel * n_pts)
        atten_memory = torch.matmul(sim, atten_memory)
        atten_memory = atten_memory.view(n_batch, n_cutout, n_channel, n_pts)

        self._memory = self._alpha * x_new + (1.0 - self._alpha) * atten_memory
        return self._memory, sim

    def _generate_neighbor_mask(self, x):
        n_cutout = x.shape[1]
        hw = int(self._window_size / 2)
        inds_col = torch.arange(n_cutout).unsqueeze(dim=-1).long()
        window_inds = torch.arange(-hw, hw + 1).long()
        inds_col = inds_col + window_inds.unsqueeze(dim=0)
        inds_col = (
            inds_col % n_cutout
            if self._panoramic_scan and not self.training
            else inds_col.clamp(min=0, max=n_cutout - 1)
        )
        inds_row = torch.arange(n_cutout).unsqueeze(dim=-1).expand_as(inds_col).long()
        inds_full = torch.stack((inds_row, inds_col), dim=2).view(-1, 2)

        masks = torch.zeros(n_cutout, n_cutout).float()
        masks[inds_full[:, 0], inds_full[:, 1]] = 1.0
        return masks.to(x.device), inds_full


class DrSpaam(nn.Module):
    def __init__(
        self,
        dropout=0.5,
        num_pts=48,
        alpha=0.5,
        embedding_length=128,
        window_size=7,
        panoramic_scan=False,
        cls_loss=None,
        mixup_alpha=0.0,
        mixup_w=0.0,
        use_box=False,
    ):
        super(DrSpaam, self).__init__()
        self.dropout = dropout
        self._use_box = use_box

        self.conv_block_1 = nn.Sequential(
            _conv1d_3(1, 64), _conv1d_3(64, 64), _conv1d_3(64, 128)
        )
        self.conv_block_2 = nn.Sequential(
            _conv1d_3(128, 128), _conv1d_3(128, 128), _conv1d_3(128, 256)
        )
        self.conv_block_3 = nn.Sequential(
            _conv1d_3(256, 256), _conv1d_3(256, 256), _conv1d_3(256, 512)
        )
        self.conv_block_4 = nn.Sequential(_conv1d_3(512, 256), _conv1d_3(256, 128))

        self.conv_cls = nn.Conv1d(128, 1, kernel_size=1)
        self.conv_reg = nn.Conv1d(128, 2, kernel_size=1)
        if use_box:
            self.conv_box = nn.Conv1d(128, 4, kernel_size=1)

        self.gate = _SpatialAttentionMemory(
            n_pts=int(ceil(num_pts / 4)),
            n_channel=256,
            embedding_length=embedding_length,
            alpha=alpha,
            window_size=window_size,
            panoramic_scan=panoramic_scan,
        )

    def forward(self, x, inference=False):
        B, CT, N, L = x.shape
        if not inference:
            self.gate.reset()

        n_scan = x.shape[2]
        for i in range(n_scan):
            x_i = x[:, :, i, :]
            out = x_i.view(B * CT, 1, L)
            out = self._conv_and_pool(out, self.conv_block_1)
            out = self._conv_and_pool(out, self.conv_block_2)
            out = out.view(B, CT, out.shape[-2], out.shape[-1])
            out, sim = self.gate(out)

        out = out.view(B * CT, out.shape[-2], out.shape[-1])
        out = self._conv_and_pool(out, self.conv_block_3)
        out = self.conv_block_4(out)
        out = F.avg_pool1d(out, kernel_size=out.shape[-1])

        pred_cls = self.conv_cls(out).view(B, CT, -1)
        pred_reg = self.conv_reg(out).view(B, CT, 2)

        if self._use_box:
            pred_box = self.conv_box(out).view(B, CT, 4)
            return pred_cls, pred_reg, pred_box, sim
        else:
            return pred_cls, pred_reg, sim

    def _conv_and_pool(self, x, conv_block):
        out = conv_block(x)
        out = F.max_pool1d(out, kernel_size=2)
        if self.dropout > 0:
            out = F.dropout(out, p=self.dropout, training=self.training)
        return out


class Detector(object):
    def __init__(
        self, ckpt_file, model="DR-SPAAM", gpu=False, stride=1, panoramic_scan=True
    ):
        self._gpu = gpu and torch.cuda.is_available()
        self._stride = stride
        self._use_dr_spaam = model == "DR-SPAAM"
        self._scan_phi = None
        self._laser_fov_deg = None

        if model == "DROW3":
            self._model = DrowNet(dropout=0.5)
        elif model == "DR-SPAAM":
            self._model = DrSpaam(
                dropout=0.5,
                num_pts=56,
                embedding_length=128,
                alpha=0.5,
                window_size=17,
                panoramic_scan=panoramic_scan,
            )
        else:
            raise NotImplementedError(f"Unsupported model: {model}")

        device = "cuda" if self._gpu else "cpu"
        ckpt = torch.load(ckpt_file, map_location=device, weights_only=False)
        self._model.load_state_dict(ckpt["model_state"])

        self._model.eval()
        if self._gpu:
            torch.backends.cudnn.benchmark = True
            self._model = self._model.cuda()

    def __call__(self, scan):
        if self._scan_phi is None or len(self._scan_phi) != len(scan):
            assert self.is_ready(), "Call set_laser_fov() first."
            half_fov_rad = 0.5 * np.deg2rad(self._laser_fov_deg)
            self._scan_phi = np.linspace(
                -half_fov_rad, half_fov_rad, len(scan), dtype=np.float32
            )

        ct = scans_to_cutout(
            scan[None, ...],
            self._scan_phi,
            stride=self._stride,
            centered=True,
            fixed=True,
            window_width=1.0,
            window_depth=0.5,
            num_cutout_pts=56,
            padding_val=29.99,
            area_mode=True,
        )
        ct = torch.from_numpy(ct).float()

        if self._gpu:
            ct = ct.cuda()

        with torch.no_grad():
            if self._use_dr_spaam:
                pred_cls, pred_reg, _ = self._model(ct.unsqueeze(dim=0), inference=True)
            else:
                pred_cls, pred_reg = self._model(ct.unsqueeze(dim=0))

        pred_cls = torch.sigmoid(pred_cls[0]).data.cpu().numpy()
        pred_reg = pred_reg[0].data.cpu().numpy()

        dets_xy, dets_cls, instance_mask = nms_predicted_center(
            scan[:: self._stride],
            self._scan_phi[:: self._stride],
            pred_cls[:, 0],
            pred_reg,
        )
        return dets_xy, dets_cls, instance_mask

    def set_laser_fov(self, fov_deg):
        self._laser_fov_deg = fov_deg

    def is_ready(self):
        return self._laser_fov_deg is not None
