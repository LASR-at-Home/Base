import torch
from torch import nn
from pointnet2_ops import pointnet2_utils

# from .Transformer import PCTransformer
from timm.models.layers import DropPath, trunc_normal_
from knn_cuda import KNN


knn_1 = KNN(k=16, transpose_mode=False)


class DGCNN_Grouper(nn.Module):
    def __init__(self):
        super().__init__()
        """
        K has to be 16
        """
        self.input_trans = nn.Conv1d(3, 8, 1)

        self.layer1 = nn.Sequential(
            nn.Conv2d(16, 32, kernel_size=1, bias=False),
            nn.GroupNorm(4, 32),
            nn.LeakyReLU(negative_slope=0.2),
        )

        self.layer2 = nn.Sequential(
            nn.Conv2d(64, 64, kernel_size=1, bias=False),
            nn.GroupNorm(4, 64),
            nn.LeakyReLU(negative_slope=0.2),
        )

        self.layer3 = nn.Sequential(
            nn.Conv2d(128, 64, kernel_size=1, bias=False),
            nn.GroupNorm(4, 64),
            nn.LeakyReLU(negative_slope=0.2),
        )

        self.layer4 = nn.Sequential(
            nn.Conv2d(128, 128, kernel_size=1, bias=False),
            nn.GroupNorm(4, 128),
            nn.LeakyReLU(negative_slope=0.2),
        )

    @staticmethod
    def fps_downsample(coor, x, num_group):
        xyz = coor.transpose(1, 2).contiguous()  # b, n, 3
        fps_idx = pointnet2_utils.furthest_point_sample(xyz, num_group)

        combined_x = torch.cat([coor, x], dim=1)

        new_combined_x = pointnet2_utils.gather_operation(combined_x, fps_idx)

        new_coor = new_combined_x[:, :3]
        new_x = new_combined_x[:, 3:]

        return new_coor, new_x

    @staticmethod
    def get_graph_feature(coor_q, x_q, coor_k, x_k):

        # coor: bs, 3, np, x: bs, c, np

        k = 16
        batch_size = x_k.size(0)
        num_points_k = x_k.size(2)
        num_points_q = x_q.size(2)

        with torch.no_grad():
            _, idx = knn_1(coor_k, coor_q)  # bs k np
            assert idx.shape[1] == k
            idx_base = (
                torch.arange(0, batch_size, device=x_q.device).view(-1, 1, 1)
                * num_points_k
            )
            idx = idx + idx_base
            idx = idx.view(-1)
        num_dims = x_k.size(1)
        x_k = x_k.transpose(2, 1).contiguous()
        feature = x_k.view(batch_size * num_points_k, -1)[idx, :]
        feature = (
            feature.view(batch_size, k, num_points_q, num_dims)
            .permute(0, 3, 2, 1)
            .contiguous()
        )
        x_q = x_q.view(batch_size, num_dims, num_points_q, 1).expand(-1, -1, -1, k)
        feature = torch.cat((feature - x_q, x_q), dim=1)
        return feature

    def forward(self, x):

        # x: bs, 3, np

        # bs 3 N(128)   bs C(224)128 N(128)
        coor = x
        f = self.input_trans(x)

        f = self.get_graph_feature(coor, f, coor, f)
        f = self.layer1(f)
        f = f.max(dim=-1, keepdim=False)[0]

        coor_q, f_q = self.fps_downsample(coor, f, 512)
        f = self.get_graph_feature(coor_q, f_q, coor, f)
        f = self.layer2(f)
        f = f.max(dim=-1, keepdim=False)[0]
        coor = coor_q

        f = self.get_graph_feature(coor, f, coor, f)
        f = self.layer3(f)
        f = f.max(dim=-1, keepdim=False)[0]

        coor_q, f_q = self.fps_downsample(coor, f, 128)
        f = self.get_graph_feature(coor_q, f_q, coor, f)
        f = self.layer4(f)
        f = f.max(dim=-1, keepdim=False)[0]
        coor = coor_q

        return coor, f


def fps(pc, num):
    fps_idx = pointnet2_utils.furthest_point_sample(pc, num)
    sub_pc = (
        pointnet2_utils.gather_operation(pc.transpose(1, 2).contiguous(), fps_idx)
        .transpose(1, 2)
        .contiguous()
    )
    return sub_pc


knn_2 = KNN(k=8, transpose_mode=False)


def get_knn_index(coor_q, coor_k=None):
    coor_k = coor_k if coor_k is not None else coor_q
    # coor: bs, 3, np
    batch_size, _, num_points = coor_q.size()
    num_points_k = coor_k.size(2)

    with torch.no_grad():
        _, idx = knn_2(coor_k, coor_q)  # bs k np
        idx_base = (
            torch.arange(0, batch_size, device=coor_q.device).view(-1, 1, 1)
            * num_points_k
        )
        idx = idx + idx_base
        idx = idx.view(-1)

    return idx


def get_graph_feature(x, knn_index, x_q=None):
    k = 8
    batch_size, num_points, num_dims = x.size()
    num_query = x_q.size(1) if x_q is not None else num_points
    feature = x.view(batch_size * num_points, num_dims)[knn_index, :]
    feature = feature.view(batch_size, k, num_query, num_dims)
    x = x_q if x_q is not None else x
    x = x.view(batch_size, 1, num_query, num_dims).expand(-1, k, -1, -1)
    feature = torch.cat((feature - x, x), dim=-1)
    return feature


class Folding(nn.Module):
    def __init__(self, in_channel, step, hidden_dim=512):
        super().__init__()

        self.in_channel = in_channel
        self.step = step

        a = (
            torch.linspace(-1.0, 1.0, steps=step, dtype=torch.float)
            .view(1, step)
            .expand(step, step)
            .reshape(1, -1)
        )
        b = (
            torch.linspace(-1.0, 1.0, steps=step, dtype=torch.float)
            .view(step, 1)
            .expand(step, step)
            .reshape(1, -1)
        )
        self.folding_seed = torch.cat([a, b], dim=0).cuda()

        self.folding1 = nn.Sequential(
            nn.Conv1d(in_channel + 2, hidden_dim, 1),
            nn.BatchNorm1d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv1d(hidden_dim, hidden_dim // 2, 1),
            nn.BatchNorm1d(hidden_dim // 2),
            nn.ReLU(inplace=True),
            nn.Conv1d(hidden_dim // 2, 3, 1),
        )

        self.folding2 = nn.Sequential(
            nn.Conv1d(in_channel + 3, hidden_dim, 1),
            nn.BatchNorm1d(hidden_dim),
            nn.ReLU(inplace=True),
            nn.Conv1d(hidden_dim, hidden_dim // 2, 1),
            nn.BatchNorm1d(hidden_dim // 2),
            nn.ReLU(inplace=True),
            nn.Conv1d(hidden_dim // 2, 3, 1),
        )

    def forward(self, x):
        num_sample = self.step * self.step
        bs = x.size(0)
        features = x.view(bs, self.in_channel, 1).expand(
            bs, self.in_channel, num_sample
        )
        seed = (
            self.folding_seed.view(1, 2, num_sample)
            .expand(bs, 2, num_sample)
            .to(x.device)
        )

        x = torch.cat([seed, features], dim=1)
        fd1 = self.folding1(x)
        x = torch.cat([fd1, features], dim=1)
        fd2 = self.folding2(x)

        return fd2


class Mlp(nn.Module):
    def __init__(
        self,
        in_features,
        hidden_features=None,
        out_features=None,
        act_layer=nn.GELU,
        drop=0.0,
    ):
        super().__init__()
        out_features = out_features or in_features
        hidden_features = hidden_features or in_features
        self.fc1 = nn.Linear(in_features, hidden_features)
        self.act = act_layer()
        self.fc2 = nn.Linear(hidden_features, out_features)
        self.drop = nn.Dropout(drop)

    def forward(self, x):
        x = self.fc1(x)
        x = self.act(x)
        x = self.drop(x)
        x = self.fc2(x)
        x = self.drop(x)
        return x


class Attention(nn.Module):
    def __init__(
        self,
        dim,
        num_heads=8,
        qkv_bias=False,
        qk_scale=None,
        attn_drop=0.0,
        proj_drop=0.0,
    ):
        super().__init__()
        self.num_heads = num_heads
        head_dim = dim // num_heads
        self.scale = qk_scale or head_dim**-0.5
        self.qkv = nn.Linear(dim, dim * 3, bias=qkv_bias)
        self.attn_drop = nn.Dropout(attn_drop)
        self.proj = nn.Linear(dim, dim)
        self.proj_drop = nn.Dropout(proj_drop)

    def forward(self, x):
        B, N, C = x.shape
        qkv = (
            self.qkv(x)
            .reshape(B, N, 3, self.num_heads, C // self.num_heads)
            .permute(2, 0, 3, 1, 4)
        )
        q, k, v = qkv[0], qkv[1], qkv[2]

        attn = (q @ k.transpose(-2, -1)) * self.scale
        attn = attn.softmax(dim=-1)
        attn = self.attn_drop(attn)

        x = (attn @ v).transpose(1, 2).reshape(B, N, C)
        # Offset-Attention
        # x = x - xx

        x = self.proj(x)
        x = self.proj_drop(x)
        return x


class CRAttention(nn.Module):
    def __init__(
        self,
        dim,
        out_dim,
        num_heads=8,
        qkv_bias=False,
        qk_scale=None,
        attn_drop=0.0,
        proj_drop=0.0,
    ):
        super().__init__()
        self.num_heads = num_heads
        self.dim = dim
        self.out_dim = out_dim
        head_dim = out_dim // num_heads
        self.scale = qk_scale or head_dim**-0.5
        self.q_map = nn.Linear(dim, out_dim, bias=qkv_bias)
        self.k_map = nn.Linear(dim, out_dim, bias=qkv_bias)
        self.v_map = nn.Linear(dim, out_dim, bias=qkv_bias)
        self.at_drop = nn.Dropout(attn_drop)
        self.proj = nn.Linear(out_dim, out_dim)
        self.proj_drop = nn.Dropout(proj_drop)

    def forward(self, q, v):
        B, N, _ = q.shape
        C = self.out_dim
        k = v
        NK = k.size(1)

        q = (
            self.q_map(q)
            .view(B, N, self.num_heads, C // self.num_heads)
            .permute(0, 2, 1, 3)
        )
        k = (
            self.k_map(k)
            .view(B, NK, self.num_heads, C // self.num_heads)
            .permute(0, 2, 1, 3)
        )
        v = (
            self.v_map(v)
            .view(B, NK, self.num_heads, C // self.num_heads)
            .permute(0, 2, 1, 3)
        )

        at = (q @ k.transpose(-2, -1)) * self.scale
        at = at.softmax(dim=-1)
        at = self.at_drop(at)

        x = (at @ v).transpose(1, 2).reshape(B, N, C)
        x = self.proj(x)
        x = self.proj_drop(x)
        return x


class DecoderNetwork(nn.Module):
    def __init__(
        self,
        dim,
        num_heads,
        dim_q=None,
        mlp_ratio=4.0,
        qkv_bias=False,
        qk_scale=None,
        drop=0.0,
        attn_drop=0.0,
        drop_path=0.0,
        act_layer=nn.GELU,
        norm_layer=nn.LayerNorm,
    ):
        super().__init__()
        self.norm1 = norm_layer(dim)
        self.self_attn = Attention(
            dim,
            num_heads=num_heads,
            qkv_bias=qkv_bias,
            qk_scale=qk_scale,
            attn_drop=attn_drop,
            proj_drop=drop,
        )
        dim_q = dim_q or dim
        self.norm_q = norm_layer(dim_q)
        self.norm_v = norm_layer(dim)
        self.attn = CRAttention(
            dim,
            dim,
            num_heads=num_heads,
            qkv_bias=qkv_bias,
            qk_scale=qk_scale,
            attn_drop=attn_drop,
            proj_drop=drop,
        )
        # NOTE: drop path for stochastic depth, we shall see if this is better than dropout here
        self.drop_path = DropPath(drop_path) if drop_path > 0.0 else nn.Identity()
        self.norm2 = norm_layer(dim)
        mlp_hidden_dim = int(dim * mlp_ratio)
        self.mlp = Mlp(
            in_features=dim,
            hidden_features=mlp_hidden_dim,
            act_layer=act_layer,
            drop=drop,
        )

        self.knn_map = nn.Sequential(
            nn.Linear(dim * 2, dim), nn.LeakyReLU(negative_slope=0.2)
        )

        self.merge_map = nn.Linear(dim * 2, dim)

        self.knn_map_cross = nn.Sequential(
            nn.Linear(dim * 2, dim), nn.LeakyReLU(negative_slope=0.2)
        )

        self.merge_map_cross = nn.Linear(dim * 2, dim)

    def forward(self, q, v, self_knn_index=None, cross_knn_index=None):
        norm_q = self.norm1(q)
        q_1 = self.self_attn(norm_q)

        if self_knn_index is not None:
            knn_f = get_graph_feature(norm_q, self_knn_index)
            knn_f = self.knn_map(knn_f)
            knn_f = knn_f.max(dim=1, keepdim=False)[0]
            q_1 = torch.cat([q_1, knn_f], dim=-1)
            q_1 = self.merge_map(q_1)

        q = q + self.drop_path(q_1)

        norm_q = self.norm_q(q)
        norm_v = self.norm_v(v)
        q_2 = self.attn(norm_q, norm_v)

        if cross_knn_index is not None:
            knn_f = get_graph_feature(norm_v, cross_knn_index, norm_q)
            knn_f = self.knn_map_cross(knn_f)
            knn_f = knn_f.max(dim=1, keepdim=False)[0]
            q_2 = torch.cat([q_2, knn_f], dim=-1)
            q_2 = self.merge_map_cross(q_2)

        q = q + self.drop_path(q_2)
        q = q + self.drop_path(self.mlp(self.norm2(q)))
        return q


class Block(nn.Module):

    def __init__(
        self,
        dim,
        num_heads,
        mlp_ratio=4.0,
        qkv_bias=False,
        qk_scale=None,
        drop=0.0,
        attn_drop=0.0,
        drop_path=0.0,
        act_layer=nn.GELU,
        norm_layer=nn.LayerNorm,
    ):
        super().__init__()
        self.norm1 = norm_layer(dim)
        self.attn = Attention(
            dim,
            num_heads=num_heads,
            qkv_bias=qkv_bias,
            qk_scale=qk_scale,
            attn_drop=attn_drop,
            proj_drop=drop,
        )
        self.drop_path = DropPath(drop_path) if drop_path > 0.0 else nn.Identity()
        self.norm2 = norm_layer(dim)
        mlp_hidden_dim = int(dim * mlp_ratio)

        self.knn_map = nn.Sequential(
            nn.Linear(dim * 2, dim), nn.LeakyReLU(negative_slope=0.2)
        )

        self.merge_map = nn.Linear(dim * 2, dim)

        self.mlp = Mlp(
            in_features=dim,
            hidden_features=mlp_hidden_dim,
            act_layer=act_layer,
            drop=drop,
        )

    def forward(self, x, knn_index=None):
        norm_x = self.norm1(x)
        x_1 = self.attn(norm_x)

        if knn_index is not None:
            knn_f = get_graph_feature(norm_x, knn_index)
            knn_f = self.knn_map(knn_f)
            knn_f = knn_f.max(dim=1, keepdim=False)[0]
            x_1 = torch.cat([x_1, knn_f], dim=-1)
            x_1 = self.merge_map(x_1)

        x = x + self.drop_path(x_1)
        x = x + self.drop_path(self.mlp(self.norm2(x)))
        return x


class PCT(nn.Module):

    def __init__(
        self,
        in_chans=3,
        embed_dim=768,
        depth=[6, 6],
        num_heads=6,
        mlp_ratio=2.0,
        qkv_bias=False,
        qk_scale=None,
        drop_rate=0.0,
        attn_drop_rate=0.0,
        num_query=224,
        knn_layer=-1,
    ):
        super().__init__()

        self.num_features = self.embed_dim = embed_dim

        self.knn_layer = knn_layer

        self.grouper = DGCNN_Grouper()

        self.pos_embed = nn.Sequential(
            nn.Conv1d(in_chans, 128, 1),
            nn.BatchNorm1d(128),
            nn.LeakyReLU(negative_slope=0.2),
            nn.Conv1d(128, embed_dim, 1),
        )

        self.input_proj = nn.Sequential(
            nn.Conv1d(128, embed_dim, 1),
            nn.BatchNorm1d(embed_dim),
            nn.LeakyReLU(negative_slope=0.2),
            nn.Conv1d(embed_dim, embed_dim, 1),
        )

        self.encoder = nn.ModuleList(
            [
                Block(
                    dim=embed_dim,
                    num_heads=num_heads,
                    mlp_ratio=mlp_ratio,
                    qkv_bias=qkv_bias,
                    qk_scale=qk_scale,
                    drop=drop_rate,
                    attn_drop=attn_drop_rate,
                )
                for i in range(depth[0])
            ]
        )

        self.increase_dim = nn.Sequential(
            nn.Conv1d(embed_dim, 1024, 1),
            nn.BatchNorm1d(1024),
            nn.LeakyReLU(negative_slope=0.2),
            nn.Conv1d(1024, 1024, 1),
        )

        self.num_query = num_query
        self.coarse_pred = nn.Sequential(
            nn.Linear(1024, 1024), nn.ReLU(inplace=True), nn.Linear(1024, 3 * num_query)
        )
        self.mlp_query = nn.Sequential(
            nn.Conv1d(1024 + 3, 1024, 1),
            # nn.BatchNorm1d(1024),
            nn.LeakyReLU(negative_slope=0.2),
            nn.Conv1d(1024, 1024, 1),
            # nn.BatchNorm1d(1024),
            nn.LeakyReLU(negative_slope=0.2),
            nn.Conv1d(1024, embed_dim, 1),
        )

        self.decoder = nn.ModuleList(
            [
                DecoderNetwork(
                    dim=embed_dim,
                    num_heads=num_heads,
                    mlp_ratio=mlp_ratio,
                    qkv_bias=qkv_bias,
                    qk_scale=qk_scale,
                    drop=drop_rate,
                    attn_drop=attn_drop_rate,
                )
                for i in range(depth[1])
            ]
        )
        self.apply(self._init_weights)

    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            trunc_normal_(m.weight, std=0.02)
            if isinstance(m, nn.Linear) and m.bias is not None:
                nn.init.constant_(m.bias, 0)
        elif isinstance(m, nn.LayerNorm):
            nn.init.constant_(m.bias, 0)
            nn.init.constant_(m.weight, 1.0)
        elif isinstance(m, nn.Conv1d):
            nn.init.xavier_normal_(m.weight.data, gain=1)
        elif isinstance(m, nn.BatchNorm1d):
            nn.init.constant_(m.weight.data, 1)
            nn.init.constant_(m.bias.data, 0)

    def forward(self, inpc):

        basi = inpc.size(0)
        xy, f = self.grouper(inpc.transpose(1, 2).contiguous())
        knn_index = get_knn_index(xy)
        pos = self.pos_embed(xy).transpose(1, 2)
        x = self.input_proj(f).transpose(1, 2)

        # encoder
        for i, enc in enumerate(self.encoder):
            if i < self.knn_layer:
                x = enc(x + pos, knn_index)  # B N C
            else:
                x = enc(x + pos)

        global_feature = self.increase_dim(x.transpose(1, 2))  # B 1024 N
        global_feature = torch.max(global_feature, dim=-1)[0]  # B 1024

        coarse_point_cloud = self.coarse_pred(global_feature).reshape(
            basi, -1, 3
        )  # B M C(3)

        new_knn_index = get_knn_index(coarse_point_cloud.transpose(1, 2).contiguous())
        cross_knn_index = get_knn_index(
            coor_k=xy, coor_q=coarse_point_cloud.transpose(1, 2).contiguous()
        )

        query_feature = torch.cat(
            [
                global_feature.unsqueeze(1).expand(-1, self.num_query, -1),
                coarse_point_cloud,
            ],
            dim=-1,
        )  # B M C+3
        q = self.mlp_query(query_feature.transpose(1, 2)).transpose(1, 2)  # B M C

        # decoder
        for i, dec in enumerate(self.decoder):
            if i < self.knn_layer:
                q = dec(q, x, new_knn_index, cross_knn_index)  # B M C
            else:
                q = dec(q, x)

        return q, coarse_point_cloud


class SGrasp(nn.Module):
    def __init__(self, config, **kwargs):
        super().__init__()
        self.trans_dim = config.trans_dim
        self.knn_layer = config.knn_layer
        self.num_pred = config.num_pred
        self.num_query = config.num_query

        self.fold_step = int(pow(self.num_pred // self.num_query, 0.5) + 0.5)
        self.base_model = PCT(
            in_chans=3,
            embed_dim=self.trans_dim,
            depth=[6, 8],
            drop_rate=0.0,
            num_query=self.num_query,
            knn_layer=self.knn_layer,
        )

        self.foldingnet = Folding(
            self.trans_dim, step=self.fold_step, hidden_dim=256
        )  # rebuild a cluster point

        self.increase_dim = nn.Sequential(
            nn.Conv1d(self.trans_dim, 1024, 1),
            nn.BatchNorm1d(1024),
            nn.LeakyReLU(negative_slope=0.2),
            nn.Conv1d(1024, 1024, 1),
        )
        self.reduce_map = nn.Linear(self.trans_dim + 1027, self.trans_dim)

    def forward(self, points):
        q, coarse_point_cloud = self.base_model(points)  # B M C and B M 3
        B, M, C = q.shape

        _feature = self.increase_dim(q.transpose(1, 2)).transpose(1, 2)
        _feature = torch.max(_feature, dim=1)[0]

        building_feature = torch.cat(
            [_feature.unsqueeze(-2).expand(-1, M, -1), q, coarse_point_cloud], dim=-1
        )
        building_feature = self.reduce_map(building_feature.reshape(B * M, -1))

        # foldingNet
        Final_xyz = self.foldingnet(building_feature).reshape(B, M, 3, -1)
        build_points = (
            (Final_xyz[:, :, :3, :] + coarse_point_cloud.unsqueeze(-1))
            .transpose(2, 3)
            .reshape(B, -1, 3)
        )

        # cat
        inp_sparse = fps(points, self.num_query)
        sparse_pcd = torch.cat([coarse_point_cloud, inp_sparse], dim=1).contiguous()
        build_points = torch.cat([build_points, points], dim=1).contiguous()
        output = (sparse_pcd, build_points)
        return output
