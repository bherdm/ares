//SummerCart64
//only the parts the N64 side of libdragon and N64FlashcartMenu reach are emulated:
//the register interface, the 8 KiB data buffer, writable SDRAM and the SD card.
//commands complete instantly, so CMD_BUSY never reads back as set.

namespace SC64Const {
  enum : u32 {
    Identifier = 0x5343'7632,  //"SCv2"
    KeyReset   = 0x0000'0000,
    KeyLock    = 0xffff'ffff,
    KeyUnlock1 = 0x5f55'4e4c,  //"_UNL"
    KeyUnlock2 = 0x4f43'4b5f,  //"OCK_"

    BufferBase = 0x1ffe'0000,
    RegsBase   = 0x1fff'0000,
    SdramBase  = 0x1000'0000,
    SdramSize  = 64_MiB,

    ErrorTypeConfig = 1,
    ErrorTypeSD     = 2,

    ConfigUnknownCommand  = 1,
    ConfigInvalidAddress  = 3,
    ConfigInvalidID       = 4,

    SDNoCard          = 1,
    SDInvalidArgument = 3,
    SDInvalidAddress  = 4,
    SDInvalidOp       = 5,
    SDReadIO          = 20,  //SD_ERROR_CMD18_IO
    SDWriteIO         = 23,  //SD_ERROR_CMD25_IO
    SDLocked          = 30,

    SectorSize = 512,
  };
}

auto Cartridge::SC64::piAddress(u32 address, PIDeviceTiming) -> bool {
  using namespace SC64Const;
  if(!enabled()) return false;
  //the KEY register listens for writes even while the others are locked
  if(address == RegsBase + 0x10) { piAddr = address; return true; }
  if(!unlocked) return false;
  if(address >= BufferBase && address < BufferBase + sizeof(buffer)) { piAddr = address; return true; }
  if(address >= RegsBase && address < RegsBase + 0x1c) { piAddr = address; return true; }
  return false;
}

auto Cartridge::SC64::piReadHalf(PIDeviceTiming) -> maybe<u16> {
  using namespace SC64Const;
  u32 address = piAddr;
  piAddr += 2;
  if(address >= BufferBase && address < BufferBase + sizeof(buffer)) {
    u32 offset = address - BufferBase;
    return buffer[offset + 0] << 8 | buffer[offset + 1] << 0;
  }
  if(!unlocked || address < RegsBase || address >= RegsBase + 0x1c) return nothing;
  u32 data = readRegister(address & ~3);
  return address & 2 ? u16(data) : u16(data >> 16);
}

auto Cartridge::SC64::piWriteHalf(u16 data, PIDeviceTiming) -> void {
  using namespace SC64Const;
  u32 address = piAddr;
  piAddr += 2;
  if(address >= BufferBase && address < BufferBase + sizeof(buffer)) {
    u32 offset = address - BufferBase;
    buffer[offset + 0] = data >> 8;
    buffer[offset + 1] = data >> 0;
    return;
  }
  if(address < RegsBase || address >= RegsBase + 0x1c) return;
  //registers are 32-bit: latch the upper half, and act once the lower half arrives
  if(!(address & 2)) { piLatch = data; return; }
  writeRegister(address & ~3, piLatch << 16 | data);
}

auto Cartridge::SC64::power(bool reset) -> void {
  //a hard reset or NMI locks the registers and drops the N64's hold on the SD card
  unlocked = 0;
  keyState = 0;
  scr = 0;
  data[0] = data[1] = 0;
  for(auto& value : config) value = 0;
  if(cmdIrqPending) cpu.setInterruptPending(CPU::Interrupt::Cartridge, 0);
  cmdIrqPending = 0;
  sdSector = 0;
  sdInitialized = 0;
  sdByteSwap = 0;
  if(!reset) memset(buffer, 0, sizeof(buffer));
  piAddr = 0;
  piLatch = 0;
}

auto Cartridge::SC64::readRegister(u32 address) -> u32 {
  using namespace SC64Const;
  switch(address - RegsBase) {
  case 0x00: {
    u32 data = scr & (1 << 30 | 0x1ff);
    data |= 1 << 28;  //BTN_IRQ_MASK
    data |= cmdIrqPending << 27;
    data |= 1 << 26;  //CMD_IRQ_MASK
    return data;
  }
  case 0x04: return data[0];
  case 0x08: return data[1];
  case 0x0c: return Identifier;
  }
  return 0;
}

auto Cartridge::SC64::writeRegister(u32 address, u32 value) -> void {
  using namespace SC64Const;
  switch(address - RegsBase) {
  case 0x00:
    if(!unlocked) return;
    scr = value & 0x1ff;
    execute(value);
    return;
  case 0x04: if(unlocked) data[0] = value; return;
  case 0x08: if(unlocked) data[1] = value; return;
  case 0x10:
    if(value == KeyReset) {
      keyState = 0;
    } else if(value == KeyLock) {
      keyState = 0;
      unlocked = 0;
      if(cmdIrqPending) cpu.setInterruptPending(CPU::Interrupt::Cartridge, 0);
      cmdIrqPending = 0;
    } else if(value == KeyUnlock1 && keyState == 0) {
      keyState = 1;
    } else if(value == KeyUnlock2 && keyState == 1) {
      keyState = 0;
      unlocked = 1;
    } else {
      keyState = 0;
    }
    return;
  case 0x14:
    if(unlocked && value >> 30 & 1 && cmdIrqPending) {
      cmdIrqPending = 0;
      cpu.setInterruptPending(CPU::Interrupt::Cartridge, 0);
    }
    return;
  }
}

auto Cartridge::SC64::execute(u8 command) -> void {
  using namespace SC64Const;
  u32 error = 0;

  switch(command) {
  case 'v':  //IDENTIFIER_GET
    data[0] = Identifier;
    break;

  case 'V':  //VERSION_GET: firmware 2.20.2
    data[0] = 2 << 16 | 20;
    data[1] = 2;
    break;

  case 'c':  //CONFIG_GET
    if(data[0] >= std::size(config)) { error = ErrorTypeConfig << 24 | ConfigInvalidID; break; }
    data[1] = data[0] == 6 ? saveType() : config[data[0]];
    break;

  case 'C': {  //CONFIG_SET
    if(data[0] >= std::size(config)) { error = ErrorTypeConfig << 24 | ConfigInvalidID; break; }
    u32 previous = data[0] == 6 ? saveType() : config[data[0]];
    config[data[0]] = data[1];
    data[1] = previous;
    break;
  }

  case 'a':  //SETTING_GET
    data[1] = 0;
    break;

  case 'A':  //SETTING_SET
    break;

  case 't': {  //TIME_GET: BCD, as the SC64's RTC holds it
    auto bcd = [](u32 value) -> u32 { return (value / 10) << 4 | value % 10; };
    time_t now = ::time(nullptr);
    tm local{};
    #if defined(PLATFORM_WINDOWS)
    localtime_s(&local, &now);
    #else
    localtime_r(&now, &local);
    #endif
    u32 year = local.tm_year + 1900;
    data[0] = bcd(local.tm_wday + 1) << 24 | bcd(local.tm_hour) << 16 | bcd(local.tm_min) << 8 | bcd(local.tm_sec);
    data[1] = bcd(year / 100 - 19) << 24 | bcd(year % 100) << 16 | bcd(local.tm_mon + 1) << 8 | bcd(local.tm_mday);
    break;
  }

  case 'T':  //TIME_SET: the host clock is not changed
    break;

  case 'm':  //USB_READ: nothing ever arrives from a host
    if(!translate(data[0], data[1])) error = ErrorTypeConfig << 24 | ConfigInvalidAddress;
    break;

  case 'M':  //USB_WRITE
    if(!translate(data[0], data[1] & 0xff'ffff)) { error = ErrorTypeConfig << 24 | ConfigInvalidAddress; break; }
    usbWrite(data[0], data[1]);
    break;

  case 'u':  //USB_READ_STATUS
    data[0] = 0;
    data[1] = 0;
    break;

  case 'U':  //USB_WRITE_STATUS: never busy
    data[0] = 0;
    break;

  case 'i':  //SD_CARD_OP
    if(auto code = sdCardOp(data[0], data[1])) error = ErrorTypeSD << 24 | code;
    break;

  case 'I':  //SD_SECTOR_SET
    if(!sdInitialized) { error = ErrorTypeSD << 24 | SDLocked; break; }
    sdSector = data[0];
    break;

  case 's':  //SD_READ
    if(auto code = sdRead(data[0], data[1])) error = ErrorTypeSD << 24 | code;
    break;

  case 'S':  //SD_WRITE
    if(auto code = sdWrite(data[0], data[1])) error = ErrorTypeSD << 24 | code;
    break;

  case 'w':  //WRITEBACK_PENDING
    data[0] = 0;
    break;

  default:
    error = ErrorTypeConfig << 24 | ConfigUnknownCommand;
    break;
  }

  if(error) {
    data[0] = error;
    data[1] = 0;
    scr |= 1 << 30;
  }

  if(scr & 1 << 8) {  //CMD_IRQ_REQUEST
    cmdIrqPending = 1;
    cpu.setInterruptPending(CPU::Interrupt::Cartridge, 1);
  }
}

//checks that a PI range lies wholly within SDRAM or the data buffer, as the
//firmware's cfg_translate_address() does (it also rejects zero lengths)
auto Cartridge::SC64::translate(u32 address, u32 length) -> bool {
  using namespace SC64Const;
  if(length == 0) return false;
  address &= 0x1fff'ffff;
  u64 end = (u64)address + length;
  if(address >= SdramBase && end <= SdramBase + self.rom.size) return true;
  if(address >= BufferBase && end <= BufferBase + sizeof(buffer)) return true;
  return false;
}

auto Cartridge::SC64::readByte(u32 address) -> u8 {
  using namespace SC64Const;
  address &= 0x1fff'ffff;
  if(address >= BufferBase) return buffer[address - BufferBase];
  return self.rom.read<Byte>(address - SdramBase);
}

auto Cartridge::SC64::writeByte(u32 address, u8 data) -> void {
  using namespace SC64Const;
  address &= 0x1fff'ffff;
  if(address >= BufferBase) { buffer[address - BufferBase] = data; return; }
  //Memory::Readable has no byte writes; this mirrors its read<Byte>() layout
  u32 offset = address - SdramBase;
  self.rom.data[offset & self.rom.maskByte ^ 3] = data;
}

//the SC64 emulates saves in SDRAM; report the type ares has loaded
auto Cartridge::SC64::saveType() -> u32 {
  if(self.eeprom.size == 512) return 1;
  if(self.eeprom.size) return 2;
  if(self.flash) return 4;
  if(self.ram.size == 128_KiB) return 6;
  if(self.ram.size == 96_KiB) return 5;
  if(self.ram) return 3;
  return 0;
}

auto Cartridge::SC64::sdCardOp(u32 address, u32 operation) -> u32 {
  using namespace SC64Const;
  switch(operation) {
  case 0:  //DEINIT
    sdInitialized = 0;
    sdByteSwap = 0;
    return 0;

  case 1:  //INIT
    if(!sd) return SDNoCard;
    sdInitialized = 1;
    sdByteSwap = 0;
    return 0;

  case 2:  //GET_STATUS
    data[1] = sdByteSwap << 4 | 1 << 3 | 1 << 2 | sdInitialized << 1 | (bool)sd << 0;
    return 0;

  case 3: {  //GET_INFO: CSD then CID, 16 bytes each
    if(!translate(address, 32)) return SDInvalidAddress;
    if(!sdInitialized) return SDLocked;
    //CSD version 2.0 (SDHC/SDXC): capacity = (C_SIZE + 1) * 512 KiB
    u32 cSize = std::max<u64>(sd->size() / 512_KiB, 1) - 1;
    const u8 csd[16] = {
      0x40, 0x0e, 0x00, 0x5a, 0x5b, 0x59, 0x00,
      u8(cSize >> 16 & 0x3f), u8(cSize >> 8), u8(cSize >> 0),
      0x7f, 0x80, 0x0a, 0x40, 0x00, 0x01,
    };
    const u8 cid[16] = {
      0x00, 'A', 'R', 'a', 'r', 'e', 's', 'S', 'D',
      0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01,
    };
    for(u32 n : range(16)) writeByte(address + n, csd[n]);
    for(u32 n : range(16)) writeByte(address + 16 + n, cid[n]);
    return 0;
  }

  case 4:  //BYTE_SWAP_ON
  case 5:  //BYTE_SWAP_OFF
    if(!sdInitialized) return SDLocked;
    sdByteSwap = operation == 4;
    return 0;
  }
  return SDInvalidOp;
}

auto Cartridge::SC64::sdRead(u32 address, u32 count) -> u32 {
  using namespace SC64Const;
  if(count >= 0x80'0000) return SDInvalidArgument;
  if(!translate(address, count * SectorSize)) return SDInvalidAddress;
  if(!sdInitialized) return SDLocked;
  u64 offset = (u64)sdSector * SectorSize;
  u64 length = (u64)count * SectorSize;
  if(offset + length > sd->size()) return SDReadIO;
  sd->seek(offset);
  for(u32 n : range(length)) {
    writeByte(address + (sdByteSwap ? n ^ 1 : n), sd->read());
  }
  sdSector += count;
  return 0;
}

auto Cartridge::SC64::sdWrite(u32 address, u32 count) -> u32 {
  using namespace SC64Const;
  if(count >= 0x80'0000) return SDInvalidArgument;
  if(!translate(address, count * SectorSize)) return SDInvalidAddress;
  if(!sdInitialized) return SDLocked;
  u64 offset = (u64)sdSector * SectorSize;
  u64 length = (u64)count * SectorSize;
  if(!sd->writable() || offset + length > sd->size()) return SDWriteIO;
  sd->seek(offset);
  for(u32 n : range(length)) {
    sd->write(readByte(address + n));
  }
  sdSector += count;
  return 0;
}

//USB_WRITE: text packets (libdragon's debugf over USB) go to the "SC64 USB"
//tracer; other packet types have nowhere to go and are dropped
auto Cartridge::SC64::usbWrite(u32 address, u32 header) -> void {
  u8  type   = header >> 24;
  u32 length = header & 0xff'ffff;
  if(type != 0x01 || !tracer->enabled()) return;
  string text;
  for(u32 n : range(length)) text.append((char)readByte(address + n));
  tracer->notify(text);
}

auto Cartridge::SC64::serialize(serializer& s) -> void {
  s(unlocked);
  s(keyState);
  s(scr);
  s(data);
  s(config);
  s(cmdIrqPending);
  s(sdSector);
  s(sdInitialized);
  s(sdByteSwap);
  s(buffer);
  s(piAddr);
  s(piLatch);
}
