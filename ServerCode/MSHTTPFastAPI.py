from typing import Any, Dict, List, Optional

import json
import os
import traceback

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

from MSFileReaderLib import MsFileReader
from win32com.client import VARIANT as variant
import pythoncom


app = FastAPI()

# 全局保存当前打开的 MsFileReader，与原来 MyHTTPServer.MSReader 作用一致
ms_reader: Optional[MsFileReader] = None


# 远程调用文件选择器所需的函数和权限控制逻辑

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")
_DEFAULT_CONFIG: Dict[str, Any] = {
    "allowed_roots": [],
}


def _load_config() -> Dict[str, Any]:
    cfg = json.loads(json.dumps(_DEFAULT_CONFIG))
    if not os.path.exists(_CONFIG_PATH):
        return cfg
    try:
        with open(_CONFIG_PATH, "r", encoding="utf-8") as fh:
            user_cfg = json.load(fh)
        if isinstance(user_cfg, dict):
            cfg.update(user_cfg)
    except Exception:
        return cfg
    return cfg


def _normalize_path(path: str) -> str:
    return os.path.normcase(os.path.realpath(os.path.abspath(path)))


def _build_allowed_roots() -> List[str]:
    cfg = _load_config()
    roots = cfg.get("allowed_roots", [])
    norm_roots: List[str] = []
    if isinstance(roots, list):
        for item in roots:
            if isinstance(item, str) and item.strip():
                norm_roots.append(_normalize_path(item))
    return norm_roots


def _is_path_allowed(path: str) -> bool:
    norm = _normalize_path(path)
    roots = _build_allowed_roots()
    for root in roots:
        root_norm = root
        if norm == root_norm:
            return True
        root_prefix = root_norm if root_norm.endswith(os.sep) else root_norm + os.sep
        if norm.startswith(root_prefix):
            return True
    return False


def _list_dir(path: str) -> Dict[str, Any]:
    if not _is_path_allowed(path):
        return {"ok": False, "error": "Path is not allowed"}

    norm_path = _normalize_path(path)
    if not os.path.exists(norm_path):
        return {"ok": False, "error": "Path does not exist"}
    if not os.path.isdir(norm_path):
        return {"ok": False, "error": "Path is not a directory"}

    dirs: List[Dict[str, str]] = []
    files: List[Dict[str, str]] = []
    try:
        with os.scandir(norm_path) as it:
            for entry in it:
                try:
                    if entry.is_dir(follow_symlinks=True):
                        dirs.append({"name": entry.name, "path": entry.path})
                    elif entry.is_file(follow_symlinks=True):
                        if entry.name.lower().endswith(".raw"):
                            files.append({"name": entry.name, "path": entry.path})
                except Exception:
                    continue
    except Exception as e:
        return {"ok": False, "error": f"Failed to list directory: {e}"}

    dirs.sort(key=lambda x: x["name"].lower())
    files.sort(key=lambda x: x["name"].lower())

    parent = os.path.dirname(norm_path)
    if parent == norm_path:
        parent = ""

    return {
        "ok": True,
        "path": norm_path,
        "parent": parent,
        "dirs": dirs,
        "files": files,
    }


class ReturnData:
    @staticmethod
    def generate_answer(origin_call: Dict[str, Any], res: Any) -> Dict[str, Any]:
        data: Dict[str, Any] = {"DataType": 2, "originCall": origin_call}
        if isinstance(res, tuple):
            data["res"] = list(res)
        else:
            data["res"] = [res]
        return data

    @staticmethod
    def generate_error(origin_call: Dict[str, Any], error_info: str) -> Dict[str, Any]:
        return {
            "DataType": 3,
            "originCall": origin_call,
            "error": error_info,
        }


def _process_get(func: str) -> Dict[str, Any]:
    global ms_reader

    args_names: List[str] = []
    args: List[Any] = []
    origin_call: Dict[str, Any] = {"argsNames": args_names, "args": args, "func": func}

    reader = ms_reader
    if reader is None:
        return ReturnData.generate_error(origin_call, "Not open files")

    try:
        if func == "info":
            file_name = reader.Reader.GetFileName()
            start_time = reader.GetStartTime()
            end_time = reader.GetEndTime()
            res = [file_name, start_time, end_time]
            return ReturnData.generate_answer(origin_call, res)
        elif func == "Close":
            # GET Close: 不带任何参数
            reader.Close()
            ms_reader = None
            return ReturnData.generate_answer(origin_call, "Close Success")
        elif hasattr(reader, func):
            # 调用包装后的 MsFileReader 方法
            res = getattr(reader, func)()
            return ReturnData.generate_answer(origin_call, res)
        else:
            # 直接调用底层 COM 对象的方法
            res = getattr(reader.Reader, func)()
            return ReturnData.generate_answer(origin_call, res)
    except Exception:
        origin_call["callType"] = "wrapped" if hasattr(ms_reader, func) else "direct"
        return ReturnData.generate_error(origin_call, traceback.format_exc())


def _process_post(func: str, post_data: Dict[str, Any]) -> Dict[str, Any]:
    global ms_reader

    args_names: List[Any] = post_data.get("argsNames", [])
    args: List[Any] = post_data.get("args", [])

    origin_call: Dict[str, Any] = {"argsNames": args_names, "args": args, "func": func}

    reader = ms_reader

    # 尚未打开文件
    if reader is None:
        if func != "Open":
            origin_call["CallType"] = "wrapped"
            return ReturnData.generate_error(origin_call, "Not open files")

        # 打开第一个文件
        try:
            assert 1 <= len(args) <= 3
            assert len(args) == len(args_names) or len(args_names) == 0

            if len(args_names) == 0:
                tmp_reader = MsFileReader(*args)
            else:
                tmp_reader = MsFileReader(**dict(zip(args_names, args)))

            origin_call["CallType"] = "wrapped"
            test_file_name = tmp_reader.Reader.GetFileName()
            if not test_file_name:
                return ReturnData.generate_error(origin_call, "Open file fails:\n No such file")

            ms_reader = tmp_reader
            return ReturnData.generate_answer(origin_call, "Open Success")
        except Exception:
            ms_reader = None
            return ReturnData.generate_error(origin_call, "Open file fails:\n{}".format(traceback.format_exc()))

    # 已经有 Reader 的情况
    reader = ms_reader

    # 特殊处理：checkMassRangeValidate 不依赖具体文件，直接调用 MsFileReader 上的静态校验函数
    if func == "checkMassRangeValidate":
        try:
            assert len(args) == 1
            mass_range = args[0]
            origin_call["CallType"] = "wrapped"
            res = MsFileReader.checkMassRangeValidate(mass_range)
            return ReturnData.generate_answer(origin_call, res)
        except Exception:
            return ReturnData.generate_error(origin_call, traceback.format_exc())

    if func == "Close":
        try:
            # Close 不应带参数
            assert len(args) == 0 and len(args_names) == 0
            reader.Close()
            ms_reader = None
            origin_call["CallType"] = "wrapped"
            return ReturnData.generate_answer(origin_call, "Close Success")
        except Exception:
            return ReturnData.generate_error(origin_call, "Close file fails:\n{}".format(traceback.format_exc()))

    if func == "Open":
        # 重新打开新文件，关闭旧文件
        try:
            assert 1 <= len(args) <= 3
            assert len(args) == len(args_names) or len(args_names) == 0

            if len(args_names) == 0:
                tmp_reader = MsFileReader(*args)
            else:
                tmp_reader = MsFileReader(**dict(zip(args_names, args)))

            origin_call["CallType"] = "wrapped"
            test_file_name = tmp_reader.Reader.GetFileName()
            if not test_file_name:
                return ReturnData.generate_error(origin_call, "Open file fails:\n No such file")

            old_reader = reader
            old_reader.Close()
            ms_reader = tmp_reader
            return ReturnData.generate_answer(origin_call, "Open Success, last file has been closed")
        except Exception:
            ms_reader = None
            return ReturnData.generate_error(origin_call, "Open file fails:\n{}".format(traceback.format_exc()))

    # 其它函数
    if func == "info":
        file_name = reader.Reader.GetFileName()
        start_time = reader.GetStartTime()
        end_time = reader.GetEndTime()
        res = [file_name, start_time, end_time]
        return ReturnData.generate_answer(origin_call, res)

    # 包装参数列表，兼容原 direct 调用中对空 list 使用 VARIANT VT_EMPTY 的处理
    direct_args: List[Any] = []
    for arg in args:
        if isinstance(arg, list) and len(arg) == 0:
            direct_args.append(variant(pythoncom.VT_EMPTY, []))
        else:
            direct_args.append(arg)

    # 无命名参数
    if len(args_names) == 0:
        if hasattr(reader, func):
            try:
                origin_call["callType"] = "wrapped"
                res = getattr(reader, func)(*args)
                return ReturnData.generate_answer(origin_call, res)
            except Exception:
                return ReturnData.generate_error(origin_call, traceback.format_exc())
        else:
            try:
                origin_call["callType"] = "direct"
                res = getattr(reader.Reader, func)(*direct_args)
                return ReturnData.generate_answer(origin_call, res)
            except Exception:
                return ReturnData.generate_error(origin_call, traceback.format_exc())

    # 有命名参数但数量不匹配
    if len(args) != len(args_names):
        return ReturnData.generate_error(
            origin_call,
            "args Names lenth({}) is not matched with args length({})".format(
                len(args_names), len(args)
            ),
        )

    # 有命名参数且数量匹配
    if hasattr(reader, func):
        try:
            origin_call["callType"] = "wrapped"
            res = getattr(reader, func)(**dict(zip(args_names, args)))
            return ReturnData.generate_answer(origin_call, res)
        except Exception:
            return ReturnData.generate_error(origin_call, traceback.format_exc())

    origin_call["callType"] = "direct"
    return ReturnData.generate_error(origin_call, "not supported function call with args names in direct mode")


# API 路由定义
@app.get("/api/fs/roots")
async def api_fs_roots() -> Dict[str, Any]:
    roots = _build_allowed_roots()
    return {
        "ok": True,
        "roots": [{"path": r, "exists": os.path.exists(r)} for r in roots],
    }


@app.post("/api/fs/roots")
async def api_fs_roots_post() -> Dict[str, Any]:
    roots = _build_allowed_roots()
    return {
        "ok": True,
        "roots": [{"path": r, "exists": os.path.exists(r)} for r in roots],
    }


@app.post("/api/fs/list")
async def api_fs_list(request: Request) -> Dict[str, Any]:
    try:
        body = await request.json()
    except Exception:
        body = {}
    path = body.get("path") if isinstance(body, dict) else None
    if not path or not isinstance(path, str):
        return {"ok": False, "error": "Path is required"}
    return _list_dir(path)


@app.post("/api/ms-chro-lite")
async def api_ms_chro_lite(request: Request) -> Dict[str, Any]:
    global ms_reader

    try:
        body = await request.json()
    except Exception:
        body = {}

    mass_range = body.get("mass_range") if isinstance(body, dict) else None
    if not mass_range or not isinstance(mass_range, str):
        return {"ok": False, "error": "mass_range is required"}

    start_time = body.get("start_time", 0) if isinstance(body, dict) else 0
    end_time = body.get("end_time", 0) if isinstance(body, dict) else 0
    try:
        start_time = float(start_time)
    except Exception:
        start_time = 0.0
    try:
        end_time = float(end_time)
    except Exception:
        end_time = 0.0
    if start_time < 0:
        start_time = 0.0
    if end_time < 0:
        end_time = 0.0

    if ms_reader is None:
        return {"ok": False, "error": "Not open files"}

    try:
        valid_flag, msg = MsFileReader.checkMassRangeValidate(mass_range)
        if not bool(valid_flag):
            return {"ok": False, "valid": False, "message": msg}

        actual_end = ms_reader.GetEndTime()
        if end_time > actual_end:
            end_time = 0.0
        if end_time != 0.0 and start_time > end_time:
            end_time = 0.0

        time_list, intensity_list, _ = ms_reader.GetChroData(
            MassRange1=mass_range,
            startTime=start_time,
            EndTime=end_time,
        )
        return {
            "ok": True,
            "valid": True,
            "mass_range": mass_range,
            "start_time": start_time,
            "end_time": end_time,
            "time": time_list,
            "intensity": intensity_list,
        }
    except Exception:
        return {"ok": False, "error": "GetChroData failed"}


@app.get("/")
async def root_get() -> JSONResponse:
    data = _process_get("info")
    print(json.dumps(data.get("originCall", {}), ensure_ascii=False))
    return JSONResponse(content=data)


@app.get("/{func_path:path}")
async def generic_get(func_path: str) -> JSONResponse:
    func = func_path or "info"
    data = _process_get(func)
    print(json.dumps(data.get("originCall", {}), ensure_ascii=False))
    return JSONResponse(content=data)


@app.post("/")
async def root_post(request: Request) -> JSONResponse:
    post_data = await request.json()
    data = _process_post("info", post_data)
    print(json.dumps(data.get("originCall", {}), ensure_ascii=False))
    return JSONResponse(content=data)


@app.post("/{func_path:path}")
async def generic_post(func_path: str, request: Request) -> JSONResponse:
    post_data = await request.json()
    func = func_path or "info"
    data = _process_post(func, post_data)
    print(json.dumps(data.get("originCall", {}), ensure_ascii=False))
    return JSONResponse(content=data)


if __name__ == "__main__":
    import uvicorn  

    # 与原脚本保持同样端口 8899
    uvicorn.run("MSHTTPFastAPI:app", host="0.0.0.0", port=8899, reload=False)
