from siyi_sdk import SiyiGimbalCamera
import asyncio

async def main():
    camera = SiyiGimbalCamera(ip="192.168.1.17")
    try:
        await camera.connect()
        if camera.is_connected:
            for i in (-50, 0, 50, 0):
                for j in (-50, 0, 50, 0):
                    await asyncio.sleep(5)
                    await camera.send_rotation_command(i, j)
            await asyncio.sleep(10)
    except Exception:
        print("Oops")
    finally:
        await camera.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
