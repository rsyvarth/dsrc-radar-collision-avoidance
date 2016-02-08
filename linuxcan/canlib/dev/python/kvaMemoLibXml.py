import ctypes as ct
import struct
import inspect
import os

KvaXmlStatusOK = 0
KvaXmlStatusFail = -1
KvaXmlStatusERR_ATTR_NOT_FOUND = -3
KvaXmlStatusERR_ATTR_VALUE = -4
KvaXmlStatusERR_ELEM_NOT_FOUND = -5
KvaXmlStatusERR_VALUE_RANGE = -6
KvaXmlStatusERR_VALUE_UNIQUE = -7
KvaXmlStatusERR_VALUE_CONSECUTIVE = -8
KvaXmlStatusERR_POSTFIXEXPR = -9
KvaXmlStatusERR_XML_PARSER = -10
KvaXmlStatusERR_DTD_VALIDATION = -11
KvaXmlStatusERR_INTERNAL = -20

#qqqmac These should be taken from DLL
kvaErrorText = {
    KvaXmlStatusOK: "OK",
    KvaXmlStatusFail: "Generic error",
    KvaXmlStatusERR_ATTR_NOT_FOUND: "Failed to find an attribute in a node",
    KvaXmlStatusERR_ATTR_VALUE: "The attribute value is not correct," +
                                " e.g. whitespace after a number.",
    KvaXmlStatusERR_ELEM_NOT_FOUND: "Could not find a required element",
    KvaXmlStatusERR_VALUE_RANGE: "The value is outside the allowed range",
    KvaXmlStatusERR_VALUE_UNIQUE: "The value is not unique; usually idx " +
                                  "attributes",
    KvaXmlStatusERR_VALUE_CONSECUTIVE: "The values are not conecutive; " +
                                       "usually idx attributes",
    KvaXmlStatusERR_POSTFIXEXPR: "The postfix expression could not be parsed",
    KvaXmlStatusERR_XML_PARSER: "The XML settings contain syntax errors.",
    KvaXmlStatusERR_DTD_VALIDATION: "The XML settings do not follow the DTD.",
    KvaXmlStatusERR_INTERNAL: "Internal errors, e.g. null pointers."
}

KvaXmlValidationStatusOK = 0
KvaXmlValidationStatusFail = -1
KvaXmlValidationStatusERR_ABORT = -2
KvaXmlValidationStatusERR_SILENT_TRANSMIT = -3
KvaXmlValidationStatusERR_UNDEFINED_TRIGGER = -4
KvaXmlValidationStatusERR_MULTIPLE_EXT_TRIGGER = -5
KvaXmlValidationStatusERR_MULTIPLE_START_TRIGGER = -6
KvaXmlValidationStatusERR_DISK_FULL_STARTS_LOG = -7
KvaXmlValidationStatusERR_NUM_OUT_OF_RANGE = -8
KvaXmlValidationStatusERR_SCRIPT_NOT_FOUND = -9
KvaXmlValidationStatusERR_SCRIPT_TOO_LARGE = -10
KvaXmlValidationStatusERR_SCRIPT_TOO_MANY = -11
KvaXmlValidationStatusERR_SCRIPT_CONFLICT = -12
KvaXmlValidationStatusERR_ELEMENT_COUNT = -13

KvaXmlValidationStatusWARN_ABORT = -100
KvaXmlValidationStatusWARN_NO_ACTIVE_LOG = -101
KvaXmlValidationStatusWARN_DISK_FULL_AND_FIFO = -102
KvaXmlValidationStatusWARN_IGNORED_ELEMENT = -103

#qqqmac These should be taken from DLL
kvaXmlValidationText = {
    KvaXmlValidationStatusOK: "OK",
    KvaXmlValidationStatusFail: "Generic error",
    KvaXmlValidationStatusERR_ABORT: "Too many errors, validation aborted",
    KvaXmlValidationStatusERR_SILENT_TRANSMIT:
        "Transmit lists used in silent mode",
    KvaXmlValidationStatusERR_UNDEFINED_TRIGGER:
        "An undefined trigger is used in an expression",
    KvaXmlValidationStatusERR_MULTIPLE_EXT_TRIGGER:
        "There are more than one external trigger defined",
    KvaXmlValidationStatusERR_MULTIPLE_START_TRIGGER:
        "There are more than one start up trigger defined",
    KvaXmlValidationStatusERR_DISK_FULL_STARTS_LOG:
        "A trigger on disk full starts the logging",
    KvaXmlValidationStatusERR_NUM_OUT_OF_RANGE:
        "A numerical value is out of range",
    KvaXmlValidationStatusERR_SCRIPT_NOT_FOUND:
        "A t-script file could not be opened",
    KvaXmlValidationStatusERR_SCRIPT_TOO_LARGE:
        "A t-script is too large for the configuration",
    KvaXmlValidationStatusERR_SCRIPT_TOO_MANY:
        "Too many active t-scripts for selected device",
    KvaXmlValidationStatusERR_SCRIPT_CONFLICT:
        "More than one active script is set as 'primary'",
    KvaXmlValidationStatusERR_ELEMENT_COUNT:
        "Too many or too few elements of this type",

    KvaXmlValidationStatusWARN_ABORT: "Too many warnings, validation aborted",
    KvaXmlValidationStatusWARN_NO_ACTIVE_LOG: "No active logging detected",
    KvaXmlValidationStatusWARN_DISK_FULL_AND_FIFO:
        "A trigger on disk full used with FIFO mode",
    KvaXmlValidationStatusWARN_IGNORED_ELEMENT: "This XML element was ignored"
}


class kvaError(Exception):
    def __init__(self, kvalib, kvaERR):
        self.kvalib = kvalib
        self.kvaERR = kvaERR

    def __kvaXmlGetLastError(self):
        msg = ct.create_string_buffer(1*1024)
        err = ct.c_int(self.kvaERR)
        self.kvalib.dll.kvaXmlGetLastError(msg, ct.sizeof(msg), ct.byref(err))
        return msg.value

    def __str__(self):
        text = "[kvaError] %s: (%d)\n" % (self.kvalib.fn,
                                          self.kvaERR)
        text += self.__kvaXmlGetLastError()
        return text


class kvaMemoLibXml(object):
    """Wrapper class for the Kvaser kvaMemoLibXml.

    This class wraps the Kvaser kvaMemoLibXml dll. For more info, see the
    kvaMemoLibXml help files which are availible in the CANlib SDK.
    http://www.kvaser.com/developer/canlib-sdk/

    """
    installDir = os.environ.get('KVDLLPATH')
    if installDir is None:
        curDir = os.path.dirname(os.path.realpath(__file__))
        baseDir = os.path.join(curDir, "..", "..")
        if 8 * struct.calcsize("P") == 32:
            installDir = os.path.join(baseDir, "bin")
        else:
            installDir = os.path.join(baseDir, "bin_x64")

    installDir = os.path.realpath(installDir)
    if not os.path.isfile(os.path.join(installDir, "kvamemolibxml.dll")):
        if os.path.isfile(os.path.join(".", "kvamemolibxml.dll")):
            installDir = "."
        else:
            raise Exception("ERROR: Expected to find kvamemolibxml.dll at %s,"
                            " set KVDLLPATH" % installDir)
    try:
        libxml2Dll = ct.WinDLL(os.path.join(installDir, 'libxml2.dll'))
        kvaMemoLibXmlDll = ct.WinDLL(os.path.join(installDir,
                                                  'kvamemolibxml.dll'))
    except Exception as e:
        print("Error loading dll from directory %s." % installDir)
        print(e)
        exit(1)

    def __init__(self):
        self.dll = kvaMemoLibXml.kvaMemoLibXmlDll
        self.dll.kvaXmlInitialize()

        self.dll.kvaXmlGetVersion.argtypes = []
        self.dll.kvaXmlGetVersion.restype = ct.c_short
        self.dll.kvaXmlGetVersion.errcheck = self._kvaErrorCheck

        self.dll.kvaXmlValidate.argtypes = [ct.c_char_p, ct.c_uint]
        self.dll.kvaXmlValidate.errcheck = self._kvaErrorCheck

        self.dll.kvaXmlGetValidationStatusCount.argtypes = [ct.POINTER(ct.c_int),
                                                            ct.POINTER(ct.c_int)]
        self.dll.kvaXmlGetValidationStatusCount.errcheck = self._kvaErrorCheck

        self.dll.kvaXmlGetValidationError.argtypes = [ct.POINTER(ct.c_int),
                                                        ct.c_char_p, ct.c_uint]
        self.dll.kvaXmlGetValidationError.errcheck = self._kvaErrorCheck

        self.dll.kvaXmlGetValidationWarning.argtypes = [ct.POINTER(ct.c_int),
                                                        ct.c_char_p, ct.c_uint]
        self.dll.kvaXmlGetValidationWarning.errcheck = self._kvaErrorCheck

        self.dll.kvaXmlGetLastError.argtypes = [ct.c_char_p, ct.c_uint,
                                                ct.POINTER(ct.c_int)]
        self.dll.kvaXmlGetLastError.errcheck = self._kvaErrorCheck

        self.dll.kvaBufferToXml.argtypes = [ct.c_char_p, ct.c_uint,
                                            ct.c_char_p, ct.POINTER(ct.c_uint),
                                            ct.POINTER(ct.c_long), ct.c_char_p]
        self.dll.kvaBufferToXml.errcheck = self._kvaErrorCheck

        self.dll.kvaXmlToBuffer.argtypes = [ct.c_char_p, ct.c_uint,
                                            ct.c_char_p, ct.POINTER(ct.c_uint),
                                            ct.POINTER(ct.c_long)]
        self.dll.kvaXmlToBuffer.errcheck = self._kvaErrorCheck

    def _kvaErrorCheck(self, result, func, arguments):
        if result < 0:
            raise kvaError(self, result)
        return result

    def getVersion(self):
        """Get the kvaMemoLibXml version number.

        Returns the version number from the kvaMemoLibXml DLL currently in use.

        Args:
            None

        Returns:
            version (string): Major.minor version number
        """
        self.fn = inspect.currentframe().f_code.co_name
        v = self.dll.kvaXmlGetVersion()
        version = "%d.%d" % (v & 0xff, v >> 8)
        return version

    def kvaBufferToXml(self, conf_lif):
        """Convert a buffer containg param.lif to XML settings.

        Scripts from the param.lif will be written to current working
        directory.

        Args:
            conf_lif (string): Buffer containing param.lif to convert.

        Returns:
            xml_buf (string): Buffer containing converted XML settings.
        """
        self.fn = inspect.currentframe().f_code.co_name
        version = ct.c_long(0)
        xml_buf = ct.create_string_buffer(500*1024)
        xml_size = ct.c_uint(ct.sizeof(xml_buf))
        script_path = ct.c_char_p("")
        self.dll.kvaBufferToXml(ct.c_char_p(conf_lif), len(conf_lif), xml_buf,
                                ct.byref(xml_size), ct.byref(version),
                                script_path)
        return xml_buf.value

    def kvaXmlToBuffer(self, conf_xml):
        """Convert XML settings to param.lif buffer.

        Args:
            conf_xml (string): XML settings to convert.

        Return:
            lif_buf (string): Buffer containing converted param.lif.
        """
        self.fn = inspect.currentframe().f_code.co_name
        version = ct.c_long(0)
        lif_buf = ct.create_string_buffer(320*32*1024)
        lif_size = ct.c_uint(ct.sizeof(lif_buf))
        self.dll.kvaXmlToBuffer(ct.c_char_p(conf_xml), len(conf_xml), lif_buf,
                                ct.byref(lif_size), ct.byref(version))
        return lif_buf.raw[:lif_size.value]

    def kvaXmlValidate(self, conf_xml):
        """Validate a buffer with XML settings.

        Args:
            conf_xml (string): string containing the XML settings to validate.

        Returns:
            countErr (int):  Number of XML validation errors.
            countWarn (int): Number of XML validation warnings.
        """
        self.fn = inspect.currentframe().f_code.co_name
        self.dll.kvaXmlValidate(ct.c_char_p(conf_xml), len(conf_xml))
        return self.xmlGetValidationStatusCount()

    def xmlGetValidationStatusCount(self):
        """Get the number of validation statuses (if any).

        Call after kvaXmlValidate().

        Returns:
            countErr (int):  Number of XML validation errors.
            countWarn (int): Number of XML validation warnings.
        """
        self.fn = inspect.currentframe().f_code.co_name
        countErr = ct.c_int(0)
        countWarn = ct.c_int(0)
        self.dll.kvaXmlGetValidationStatusCount(ct.byref(countErr),
                                                ct.byref(countWarn))
        return (countErr.value, countWarn.value)

    def xmlGetValidationError(self):
        """Get validation errors (if any).

        Call after kvaXmlValidate() until return status is
        KvaXmlValidationStatusOK.

        Returns:
            validationStatus (int): Validation status code.
            text (string):          Validation status message.
        """
        self.fn = inspect.currentframe().f_code.co_name
        validationStatus = ct.c_int(-666)
        text = ct.create_string_buffer(1048)
        self.dll.kvaXmlGetValidationError(ct.byref(validationStatus),
                                          text, len(text))
        return (validationStatus.value, text.value)

    def xmlGetValidationWarning(self):
        """Get validation warnings (if any).

        Call after kvaXmlValidate() until return status is
        KvaXmlValidationStatusOK.

        Returns:
            validationStatus (int): Validation status code.
            text (string):          Validation status message.
        """
        self.fn = inspect.currentframe().f_code.co_name
        validationStatus = ct.c_int(-666)
        text = ct.create_string_buffer(1048)
        self.dll.kvaXmlGetValidationWarning(ct.byref(validationStatus),
                                            text, len(text))
        return (validationStatus.value, text.value)

    def xmlGetLastError(self, kvaERR):
        """Get the last error message (if any).

        Get the last error message (if any) from conversion in human redable
        format.

        Args:
            kvaERR (int): kvaMemoLibXml error code.

        Returns:
           msg (string): Error message associated with kvaERR.

        """
        self.fn = inspect.currentframe().f_code.co_name
        msg = ct.create_string_buffer(1*1024)
        err = ct.c_int(kvaERR)
        self.kvalib.dll.kvaXmlGetLastError(msg, ct.sizeof(msg), ct.byref(err))
        return msg.value
