# -*- mode: python -*-

block_cipher = None


a = Analysis(['button_run.py'],
             pathex=['/home/dongho/Desktop/seldat_work/button_call'],
             binaries=[],
             datas=[],
             hiddenimports=[],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,
          name='button_run',
          debug=False,
          strip=False,
          upx=True,
          runtime_tmpdir=None,
          console=True )
