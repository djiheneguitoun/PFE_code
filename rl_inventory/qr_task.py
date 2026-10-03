"""QR des cartons : trouve les cartons de l'entrepôt, y colle un QR unique (2 faces) et donne la position de chaque QR.

Utilisé par env.py (positions pour la règle « QR lu »), tests/test_qr_placement.py et diag/test_qr_attach.py (pose des images).
Les images sont écrites dans assets/qr/ (CARTON_0000.png, CARTON_0001.png… une par carton).
"""

import os
import re

import qrcode
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

QR_DIR = os.path.join(os.path.dirname(__file__), "assets", "qr")  # dossier des images QR générées
CARTON_RE = re.compile(r"SM_CardBox", re.I)  # motif du nom des cartons dans le fichier de l'entrepôt
FACES = [(0, 1), (0, -1)]  # (axe, sens) des faces à QR : avant / arrière (axe X local) — pas gauche/droite


def find_cartons(stage):
    """Renvoie la liste des cartons de la scène (objets Xform dont le nom contient SM_CardBox)."""
    return [p for p in stage.Traverse() if p.GetTypeName() == "Xform" and CARTON_RE.search(p.GetName())]


def _make_qr_png(data, path):
    """Génère l'image PNG (RGB) d'un QR qui encode le texte data (l'identifiant du carton) et l'écrit dans path."""
    qrcode.make(data).save(path)
    Image.open(path).convert("RGB").save(path)


def _qr_material(stage, mat_path, png):
    """Crée et renvoie le matériau qui affiche l'image QR (couleur et émission tirées de l'image : lisible même mal éclairé)."""
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    tex = UsdShade.Shader.Define(stage, mat_path + "/DiffuseTexture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(png))
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    streader = UsdShade.Shader.Define(stage, mat_path + "/STReader")
    streader.CreateIdAttr("UsdPrimvarReader_float2")
    streader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(streader.ConnectableAPI(), "result")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _add_face_quad(stage, base_path, idx, lo, hi, axis, sign, mat):
    """Colle sur une face du carton un panneau carré QRTag_<idx> portant le QR (côté = 80 % du plus petit côté de la face)."""
    quad = UsdGeom.Mesh.Define(stage, f"{base_path}/QRTag_{idx}")
    a0, a1 = [i for i in range(3) if i != axis]
    c0, c1 = (lo[a0] + hi[a0]) / 2.0, (lo[a1] + hi[a1]) / 2.0
    half = 0.4 * min(hi[a0] - lo[a0], hi[a1] - lo[a1])
    # panneau légèrement décollé de la face (0,01 en unités du carton) pour ne pas se confondre avec elle
    coord = (hi[axis] if sign > 0 else lo[axis]) + sign * 0.01

    def pt(d0, d1):
        """Renvoie le sommet du panneau situé à (d0, d1) demi-côtés du centre de la face."""
        p = [0.0, 0.0, 0.0]
        p[axis], p[a0], p[a1] = coord, c0 + d0 * half, c1 + d1 * half
        return Gf.Vec3f(*p)

    # sur la face opposée, sommets pris en miroir : l'image vue de l'extérieur n'est pas inversée
    pts = [pt(-1, -1), pt(1, -1), pt(1, 1), pt(-1, 1)] if sign > 0 else [pt(1, -1), pt(-1, -1), pt(-1, 1), pt(1, 1)]
    quad.CreatePointsAttr(pts)
    quad.CreateFaceVertexCountsAttr([4])
    quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    normal = [0.0, 0.0, 0.0]
    normal[axis] = float(sign)
    quad.CreateNormalsAttr([Gf.Vec3f(*normal)] * 4)
    quad.SetNormalsInterpolation("faceVarying")
    quad.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(quad).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    UsdShade.MaterialBindingAPI.Apply(quad.GetPrim()).Bind(mat)


def _attach_qr(stage, prim, png):
    """Colle le QR d'un carton sur ses 2 faces avant et arrière (ne fait rien si c'est déjà fait)."""
    base = prim.GetPath().pathString
    if stage.GetPrimAtPath(base + "/QRTag_0").IsValid():
        return
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    mat = _qr_material(stage, base + "/QRMat", png)
    for idx, (axis, sign) in enumerate(FACES):
        _add_face_quad(stage, base, idx, lo, hi, axis, sign, mat)


def carton_qr_world_poses(stage, cartons):
    """Renvoie (positions, normales) en repère monde : centre et normale de chaque face à QR, 2 par carton, dans l'ordre des cartons."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    xcache = UsdGeom.XformCache()
    positions, normals = [], []
    for prim in cartons:
        rng = cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
        lo, hi = rng.GetMin(), rng.GetMax()
        m = xcache.GetLocalToWorldTransform(prim)
        for axis, sign in FACES:
            others = [i for i in range(3) if i != axis]
            local = [0.0, 0.0, 0.0]
            local[axis] = hi[axis] if sign > 0 else lo[axis]
            local[others[0]] = (lo[others[0]] + hi[others[0]]) / 2.0
            local[others[1]] = (lo[others[1]] + hi[others[1]]) / 2.0
            wp = m.Transform(Gf.Vec3d(*local))
            d = [0.0, 0.0, 0.0]
            d[axis] = float(sign)
            wn = m.TransformDir(Gf.Vec3d(*d)).GetNormalized()
            positions.append([wp[0], wp[1], wp[2]])
            normals.append([wn[0], wn[1], wn[2]])
    return positions, normals


def attach_qr_to_cartons(stage, limit=None):
    """Génère et colle un QR unique (CARTON_0000…) sur chaque carton, ou les limit premiers ; renvoie {identifiant: chemin du carton}."""
    os.makedirs(QR_DIR, exist_ok=True)
    cartons = find_cartons(stage)
    if limit:
        cartons = cartons[:limit]
    info = {}
    for i, prim in enumerate(cartons):
        box_id = f"CARTON_{i:04d}"
        png = os.path.join(QR_DIR, f"{box_id}.png")
        _make_qr_png(box_id, png)
        _attach_qr(stage, prim, png)
        info[box_id] = prim.GetPath().pathString
    return info
