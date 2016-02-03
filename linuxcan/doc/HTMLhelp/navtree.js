var NAVTREE =
[
  [ "Kvaser Linux CANLIB", "index.html", [
    [ "Welcome to Kvaser Linux CANLIB!", "index.html", [
      [ "Help File Overview", "page_help_file_overview.html", null ],
      [ "License and Copyright", "page_license_and_copyright.html", null ],
      [ "CANLIB API Calls Grouped by Function", "page_canlib_api_calls_grouped_by_function.html", null ],
      [ "CANLIB Core API Calls", "page_core_api_calls.html", null ]
    ] ],
    [ "Related Pages", "pages.html", [
      [ "User's Guide", "page_user_guide.html", [
        [ "Introduction", "page_user_guide_intro.html", [
          [ "What is CANLIB?", "page_user_guide_intro_what.html", null ],
          [ "Hello, CAN!", "page_user_guide_intro_hello.html", null ],
          [ "Programmer's Overview", "page_user_guide_intro_programmers.html", null ]
        ] ],
        [ "Initialization", "page_user_guide_init.html", [
          [ "Library Initialization", "page_user_guide_init_lib_init.html", null ],
          [ "Library Deinitialization and Cleanup", "page_user_guide_init_lib_deinit.html", null ],
          [ "Chips and Channels", "page_user_guide_init_sel_channel.html", null ],
          [ "Bit Rate and Other Bus Parameters", "page_user_guide_init_bit_rate.html", null ],
          [ "CAN Driver Modes", "page_user_guide_init_driver_modes.html", null ]
        ] ],
        [ "Sending and Receiving", "page_user_guide_send_recv.html", [
          [ "Bus On / Bus Off", "page_user_guide_send_recv_bus_on_off.html", null ],
          [ "Reading Messages", "page_user_guide_send_recv_reading.html", null ],
          [ "Sending Messages", "page_user_guide_send_recv_sending.html", null ],
          [ "Message Filters", "page_user_guide_send_recv_filters.html", null ],
          [ "Overruns", "page_user_guide_send_recv_overruns.html", null ],
          [ "Message Queue and Buffer Sizes", "page_user_guide_send_recv_queue_and_buf_sizes.html", null ],
          [ "Object Buffers", "page_user_guide_send_recv_obj_buf.html", null ],
          [ "Different CAN Frame Types", "page_user_guide_send_recv_sending_different_types.html", null ]
        ] ],
        [ "Handling Bus Errors", "page_user_guide_bus_errors.html", [
          [ "Handling Bus Errors", "page_user_guide_bus_errors_error_frames.html", null ],
          [ "SJA1000 Error Codes", "page_user_guide_bus_errors_sja1000_error_codes.html", null ]
        ] ],
        [ "Using Threads", "page_user_guide_threads.html", [
          [ "Threaded Applications", "page_user_guide_threads_applications.html", null ]
        ] ],
        [ "Getting and Setting Device Information", "page_user_guide_dev_info.html", [
          [ "Obtaining special information", "page_user_guide_dev_info_special.html", null ],
          [ "Obtaining Status Information", "page_user_guide_dev_info_status.html", null ]
        ] ],
        [ "Virtual Channels", "page_user_guide_virtual.html", [
          [ "Virtual Channels", "page_user_guide_virtual_info.html", null ]
        ] ],
        [ "Time Measurement", "page_user_guide_time.html", [
          [ "Time Measurement", "page_user_guide_time_general.html", null ],
          [ "Time Stamping Accuracy and Resolution", "page_user_guide_time_accuracy_and_resolution.html", null ]
        ] ],
        [ "LIN", "page_user_guide_lin.html", [
          [ "Using the LIN Bus", "page_user_guide_lin_intro.html", null ]
        ] ],
        [ "Frequently Asked Questions", "page_user_guide_faq.html", null ],
        [ "Support", "page_user_guide_support.html", null ],
        [ "Miscellaneous Topics", "page_user_guide_misc.html", [
          [ "Message Flags", "page_user_guide_misc_message_flags.html", null ],
          [ "Bit Rate Constants", "page_user_guide_misc_bitrate.html", null ],
          [ "Code and Mask Format", "page_user_guide_misc_code_and_mask.html", null ]
        ] ]
      ] ],
      [ "Message Mailboxes", "page_user_guide_send_recv_mailboxes.html", null ],
      [ "Accessing the Databases", "page_user_guide_lin_candb.html", null ],
      [ "Code snippets", "page_code_snippets.html", [
        [ "Code Examples", "page_code_snippets_examples.html", null ],
        [ "Bit Rate Examples", "page_code_snippets_bit_rate.html", null ]
      ] ],
      [ "Hardware Specific Notes", "page_hardware_specific.html", [
        [ "CAN Controller Specific Notes", "page_hardware_specific_can_controllers.html", null ]
      ] ]
    ] ],
    [ "Modules", "modules.html", [
      [ "General", "group___general.html", [
        [ "Functions", "group___general.html", [
          [ "canGetChannelData", "group___general.html#gab9552d1a588b0dbc144b097acba017b2", null ],
          [ "canGetErrorText", "group___general.html#ga01a7a415c95c579750bcdd95a1d245c4", null ],
          [ "canGetNumberOfChannels", "group___general.html#ga65169ca633cd30aa92b8a80e28a5378b", null ],
          [ "canGetVersion", "group___general.html#gafb5688859c56ecb6f8d85705d3ec2f14", null ],
          [ "canInitializeLibrary", "group___general.html#gaff1ec1d3416d3bdd56336a7b9ac008b1", null ],
          [ "canIoCtl", "group___general.html#gaeaa24db97af22478ca51d48636c7bb12", null ],
          [ "kvSetNotifyCallback", "group___general.html#ga99976c5b8e2c534b27bf9ec2e715d8d3", null ]
        ] ]
      ] ],
      [ "CAN", "group___c_a_n.html", [
        [ "Typedefs", "group___c_a_n.html", [
          [ "canNotifyData", "group___c_a_n.html#ga58db5be8859c14cd8a0c0f6963c64f26", null ]
        ] ],
        [ "Functions", "group___c_a_n.html", [
          [ "canAccept", "group___c_a_n.html#gaa4ffd2fad1932ad5763c2c923a1a12d8", null ],
          [ "canBusOff", "group___c_a_n.html#gaf1786cfbfd542b18b9c599d278837bd9", null ],
          [ "canBusOn", "group___c_a_n.html#ga99c7c99cc71580f8099a1407f4f9ea1a", null ],
          [ "canClose", "group___c_a_n.html#ga49525373a4d08d93c651ec10f79dd36b", null ],
          [ "canFlushReceiveQueue", "group___c_a_n.html#ga7abcf8f512da6ab568850b9faa0bc28b", null ],
          [ "canFlushTransmitQueue", "group___c_a_n.html#ga52a17bb8012bf025d127b4d29e5f3aa8", null ],
          [ "canGetBusOutputControl", "group___c_a_n.html#ga1683419d56af8afdbfc9184678c82fdd", null ],
          [ "canGetBusParams", "group___c_a_n.html#gaf2a734c0486030610389331685a3827a", null ],
          [ "canGetBusParamsFd", "group___c_a_n.html#gab620a5cd6249e2e53cf2a0099e7e8685", null ],
          [ "canGetRawHandle", "group___c_a_n.html#gaa1229ae7635c051a8f7fe545d9d8007a", null ],
          [ "canOpenChannel", "group___c_a_n.html#gac377d182232fb4ec2fed881c2b9ab300", null ],
          [ "canRead", "group___c_a_n.html#ga157d49a9343dea68ac953145e02266d8", null ],
          [ "canReadErrorCounters", "group___c_a_n.html#gadb3a712bd123317043cb73e1008075c8", null ],
          [ "canReadStatus", "group___c_a_n.html#gaca3da209fe673b3838a50a3abe831855", null ],
          [ "canReadSync", "group___c_a_n.html#ga16356f4d646240ff2e640773d21f4d76", null ],
          [ "canReadTimer", "group___c_a_n.html#ga04c2f80a23b992bf17591749192e8d48", null ],
          [ "canReadWait", "group___c_a_n.html#gac01f98e282609b5f6aaf2b1eabfb83ec", null ],
          [ "canResetBus", "group___c_a_n.html#gaecc8b56e75cf68548af53b2d432079af", null ],
          [ "canSetBusOutputControl", "group___c_a_n.html#gafca31590718ac7101d065b0c18b2410e", null ],
          [ "canSetBusParams", "group___c_a_n.html#ga7eb8c2e92cfae57e7ec5031818524301", null ],
          [ "canSetBusParamsC200", "group___c_a_n.html#ga1a3ba0ef73eee6b97e886730ac696935", null ],
          [ "canSetBusParamsFd", "group___c_a_n.html#gafcd85fbac103dcb123f4cd609be6fa14", null ],
          [ "canSetNotify", "group___c_a_n.html#gaa5dd0f277c7059169055321fbda87486", null ],
          [ "canTranslateBaud", "group___c_a_n.html#gaf38b95fce4930347d9986887ec046e13", null ],
          [ "canWrite", "group___c_a_n.html#ga62c185329d6741c90102511e2f37983e", null ],
          [ "canWriteSync", "group___c_a_n.html#ga304cb3a7bc2874c1f8ad361a911bcd5f", null ],
          [ "canWriteWait", "group___c_a_n.html#ga47d03bfcc31e290284e65211b61f15f3", null ]
        ] ],
        [ "Data Structures", "group___c_a_n.html", [
          [ "canNotifyData", "structcan_notify_data.html", null ]
        ] ]
      ] ],
      [ "Object buffers", "group___object_buffers.html", [
        [ "Functions", "group___object_buffers.html", [
          [ "canObjBufAllocate", "group___object_buffers.html#gaa189a35c78004d037eed4bd0c2bfa3ee", null ],
          [ "canObjBufDisable", "group___object_buffers.html#gab1238b563ecf4523092ebe561ece87ea", null ],
          [ "canObjBufEnable", "group___object_buffers.html#ga1ff3e82f6d0e9795a831e22183c6e7ec", null ],
          [ "canObjBufFree", "group___object_buffers.html#ga7353b3671b897e1f33b88f9084857382", null ],
          [ "canObjBufFreeAll", "group___object_buffers.html#gab299ecf20aa368b8ee253ba9610dff3b", null ],
          [ "canObjBufSendBurst", "group___object_buffers.html#gae3e27cd339700f26897648895e1b37a0", null ],
          [ "canObjBufSetFilter", "group___object_buffers.html#gaccca9d669c981e910c1805614ee40e72", null ],
          [ "canObjBufSetFlags", "group___object_buffers.html#ga9369c2f47886d9f815fe5513d6f5b60b", null ],
          [ "canObjBufSetMsgCount", "group___object_buffers.html#ga769ce97c3b7f3a8e246f872d7dbafe54", null ],
          [ "canObjBufSetPeriod", "group___object_buffers.html#gaa23baa37921bf089d9123eb97f32541b", null ],
          [ "canObjBufWrite", "group___object_buffers.html#gad72611f11b4947c96c8d0b50f59b2173", null ]
        ] ]
      ] ],
      [ "LIN", "group___l_i_n.html", [
        [ "Defines", "group___l_i_n.html", [
          [ "CompilerAssert", "group___l_i_n.html#ga147705d4bcf2fbb603385ec0dea71592", null ],
          [ "LIN_BIT_ERROR", "group___l_i_n.html#ga719cb4b7b29a67803d34ec60ea5e8a4d", null ],
          [ "LIN_CSUM_ERROR", "group___l_i_n.html#gaf7386e03bb99718fd308013450641cce", null ],
          [ "LIN_ENHANCED_CHECKSUM", "group___l_i_n.html#ga937225a25c3c24efabf7c8438ccecd09", null ],
          [ "LIN_MASTER", "group___l_i_n.html#gaddf7881b12723497542ff0f66222c46e", null ],
          [ "LIN_MSG_DISTURB_CSUM", "group___l_i_n.html#ga479885d0466fcaff2cb80dedd09a7805", null ],
          [ "LIN_MSG_DISTURB_PARITY", "group___l_i_n.html#gad4e1ec16e25ca05d694d1faff1ebf147", null ],
          [ "LIN_MSG_USE_ENHANCED_PARITY", "group___l_i_n.html#ga8c2ab5dd117f814cef07f6b072aaafc9", null ],
          [ "LIN_MSG_USE_STANDARD_PARITY", "group___l_i_n.html#ga88ace2b1c8f8f6ed08714fc8725a03f8", null ],
          [ "LIN_NODATA", "group___l_i_n.html#gaa7796bf62c2772d49bd5572e7302f55e", null ],
          [ "LIN_PARITY_ERROR", "group___l_i_n.html#ga79d31160f0025e7c6c7fe6d5cb843847", null ],
          [ "LIN_RX", "group___l_i_n.html#ga8a77e3db8950ff23e2a154d3c347f307", null ],
          [ "LIN_SLAVE", "group___l_i_n.html#ga9b19af88006130b3220a4ecb57cd4e0b", null ],
          [ "LIN_SYNCH_ERROR", "group___l_i_n.html#gad8f162919711144a9cc1ae398f569e53", null ],
          [ "LIN_TX", "group___l_i_n.html#gaa71a0b1071a3246b40ed830f32ed43ce", null ],
          [ "LIN_VARIABLE_DLC", "group___l_i_n.html#ga2edd31e15abb5f7cb8dd213fd707dfff", null ],
          [ "LIN_WAKEUP_FRAME", "group___l_i_n.html#ga1837b69bf93680cfe775253ebb01e5ef", null ],
          [ "LINERROR", "group___l_i_n.html#gab873580aa6b650d641cc2439b96a118a", null ],
          [ "linINVALID_HANDLE", "group___l_i_n.html#gaf080de594ae6f3a531e9b530577e9ebe", null ],
          [ "LINLIBAPI", "group___l_i_n.html#ga4817ac11ca69f5237588c0a3f422bcdb", null ]
        ] ],
        [ "Typedefs", "group___l_i_n.html", [
          [ "LinHandle", "group___l_i_n.html#ga759b2696d97bd97008d8df007d9ac44a", null ]
        ] ],
        [ "Enumerations", "group___l_i_n.html", [
          [ "LinStatus", "group___l_i_n.html#ga7a5ecfd2846ddd76cd49fb4edec7fc14", null ]
        ] ],
        [ "Functions", "group___l_i_n.html", [
          [ "linBusOff", "group___l_i_n.html#ga051ffc0c24d6322825cbc8ff21e50744", null ],
          [ "linBusOn", "group___l_i_n.html#ga79ab73655c1749ad9fe2b784885e2dd9", null ],
          [ "linClearMessage", "group___l_i_n.html#ga905647480dd89a7225e4b8ae0d82cb92", null ],
          [ "linClose", "group___l_i_n.html#ga5d8cb59baccdefc9e772ad34c01c596f", null ],
          [ "linGetCanHandle", "group___l_i_n.html#gadf904a2ba0101ac6dc622b6035cf0f5f", null ],
          [ "linGetFirmwareVersion", "group___l_i_n.html#gafa260028be850e70a99c1b0706679583", null ],
          [ "linGetTransceiverData", "group___l_i_n.html#ga95408cd6c8639514b4be8e188bd7b38a", null ],
          [ "linInitializeLibrary", "group___l_i_n.html#gaf47a1f1078f6b3919e2b2d9dfd559d8b", null ],
          [ "linOpenChannel", "group___l_i_n.html#ga040336f8176a10cb9578b47c42baef6b", null ],
          [ "linReadMessage", "group___l_i_n.html#gaca2d874c870f16c11a4e8e158817d8bf", null ],
          [ "linReadMessageWait", "group___l_i_n.html#gaa2f729a931bf644ce62b373ab7414250", null ],
          [ "linReadTimer", "group___l_i_n.html#ga8af35ecbb1aca56baed27990f3d43d4b", null ],
          [ "linRequestMessage", "group___l_i_n.html#ga068419b8b624d8918720a8907c4f9274", null ],
          [ "linSetBitrate", "group___l_i_n.html#ga77e1463234ee6c67a71a2ab57f578b7f", null ],
          [ "linSetupIllegalMessage", "group___l_i_n.html#ga1c89e03300af644cee54861f92ae567e", null ],
          [ "linSetupLIN", "group___l_i_n.html#ga911287175a2ca5574a50d17b698b6d9d", null ],
          [ "linUpdateMessage", "group___l_i_n.html#ga5bf84820248e95fde2718fa46304a5a5", null ],
          [ "linWriteMessage", "group___l_i_n.html#gac012f34a621bc885bd582398c3d5d175", null ],
          [ "linWriteSync", "group___l_i_n.html#ga1bd437b46f5923f05905c43cd4a1617a", null ],
          [ "linWriteWakeup", "group___l_i_n.html#ga4ba0a5256a785f3cc67a5e661837223e", null ]
        ] ],
        [ "Data Structures", "group___l_i_n.html", [
          [ "LinMessageInfo", "struct_lin_message_info.html", null ]
        ] ]
      ] ]
    ] ],
    [ "Data Structures", "annotated.html", [
      [ "canNotifyData", "structcan_notify_data.html", null ],
      [ "canUserIoPortData", "structcan_user_io_port_data.html", null ],
      [ "LinMessageInfo", "struct_lin_message_info.html", null ]
    ] ],
    [ "Data Structure Index", "classes.html", null ],
    [ "Data Fields", "functions.html", null ],
    [ "File List", "files.html", [
      [ "canlib.h", "canlib_8h.html", null ],
      [ "canstat.h", "canstat_8h.html", null ],
      [ "linlib.h", "linlib_8h.html", null ]
    ] ],
    [ "Examples", "examples.html", [
      [ "busparms.c", "busparms_8c-example.html", null ],
      [ "cancount.c", "cancount_8c-example.html", null ],
      [ "canmonitor.c", "canmonitor_8c-example.html", null ],
      [ "listChannels.c", "list_channels_8c-example.html", null ],
      [ "opentest.c", "opentest_8c-example.html", null ],
      [ "readTimerTest.c", "read_timer_test_8c-example.html", null ],
      [ "simplewrite.c", "simplewrite_8c-example.html", null ],
      [ "writeloop.c", "writeloop_8c-example.html", null ]
    ] ],
    [ "Globals", "globals.html", null ]
  ] ]
];

function createIndent(o,domNode,node,level)
{
  if (node.parentNode && node.parentNode.parentNode)
  {
    createIndent(o,domNode,node.parentNode,level+1);
  }
  var imgNode = document.createElement("img");
  if (level==0 && node.childrenData)
  {
    node.plus_img = imgNode;
    node.expandToggle = document.createElement("a");
    node.expandToggle.href = "javascript:void(0)";
    node.expandToggle.onclick = function() 
    {
      if (node.expanded) 
      {
        $(node.getChildrenUL()).slideUp("fast");
        if (node.isLast)
        {
          node.plus_img.src = node.relpath+"ftv2plastnode.png";
        }
        else
        {
          node.plus_img.src = node.relpath+"ftv2pnode.png";
        }
        node.expanded = false;
      } 
      else 
      {
        expandNode(o, node, false);
      }
    }
    node.expandToggle.appendChild(imgNode);
    domNode.appendChild(node.expandToggle);
  }
  else
  {
    domNode.appendChild(imgNode);
  }
  if (level==0)
  {
    if (node.isLast)
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2plastnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2lastnode.png";
        domNode.appendChild(imgNode);
      }
    }
    else
    {
      if (node.childrenData)
      {
        imgNode.src = node.relpath+"ftv2pnode.png";
      }
      else
      {
        imgNode.src = node.relpath+"ftv2node.png";
        domNode.appendChild(imgNode);
      }
    }
  }
  else
  {
    if (node.isLast)
    {
      imgNode.src = node.relpath+"ftv2blank.png";
    }
    else
    {
      imgNode.src = node.relpath+"ftv2vertline.png";
    }
  }
  imgNode.border = "0";
}

function newNode(o, po, text, link, childrenData, lastNode)
{
  var node = new Object();
  node.children = Array();
  node.childrenData = childrenData;
  node.depth = po.depth + 1;
  node.relpath = po.relpath;
  node.isLast = lastNode;

  node.li = document.createElement("li");
  po.getChildrenUL().appendChild(node.li);
  node.parentNode = po;

  node.itemDiv = document.createElement("div");
  node.itemDiv.className = "item";

  node.labelSpan = document.createElement("span");
  node.labelSpan.className = "label";

  createIndent(o,node.itemDiv,node,0);
  node.itemDiv.appendChild(node.labelSpan);
  node.li.appendChild(node.itemDiv);

  var a = document.createElement("a");
  node.labelSpan.appendChild(a);
  node.label = document.createTextNode(text);
  a.appendChild(node.label);
  if (link) 
  {
    a.href = node.relpath+link;
  } 
  else 
  {
    if (childrenData != null) 
    {
      a.className = "nolink";
      a.href = "javascript:void(0)";
      a.onclick = node.expandToggle.onclick;
      node.expanded = false;
    }
  }

  node.childrenUL = null;
  node.getChildrenUL = function() 
  {
    if (!node.childrenUL) 
    {
      node.childrenUL = document.createElement("ul");
      node.childrenUL.className = "children_ul";
      node.childrenUL.style.display = "none";
      node.li.appendChild(node.childrenUL);
    }
    return node.childrenUL;
  };

  return node;
}

function showRoot()
{
  var headerHeight = $("#top").height();
  var footerHeight = $("#nav-path").height();
  var windowHeight = $(window).height() - headerHeight - footerHeight;
  navtree.scrollTo('#selected',0,{offset:-windowHeight/2});
}

function expandNode(o, node, imm)
{
  if (node.childrenData && !node.expanded) 
  {
    if (!node.childrenVisited) 
    {
      getNode(o, node);
    }
    if (imm)
    {
      $(node.getChildrenUL()).show();
    } 
    else 
    {
      $(node.getChildrenUL()).slideDown("fast",showRoot);
    }
    if (node.isLast)
    {
      node.plus_img.src = node.relpath+"ftv2mlastnode.png";
    }
    else
    {
      node.plus_img.src = node.relpath+"ftv2mnode.png";
    }
    node.expanded = true;
  }
}

function getNode(o, po)
{
  po.childrenVisited = true;
  var l = po.childrenData.length-1;
  for (var i in po.childrenData) 
  {
    var nodeData = po.childrenData[i];
    po.children[i] = newNode(o, po, nodeData[0], nodeData[1], nodeData[2],
        i==l);
  }
}

function findNavTreePage(url, data)
{
  var nodes = data;
  var result = null;
  for (var i in nodes) 
  {
    var d = nodes[i];
    if (d[1] == url) 
    {
      return new Array(i);
    }
    else if (d[2] != null) // array of children
    {
      result = findNavTreePage(url, d[2]);
      if (result != null) 
      {
        return (new Array(i).concat(result));
      }
    }
  }
  return null;
}

function initNavTree(toroot,relpath)
{
  var o = new Object();
  o.toroot = toroot;
  o.node = new Object();
  o.node.li = document.getElementById("nav-tree-contents");
  o.node.childrenData = NAVTREE;
  o.node.children = new Array();
  o.node.childrenUL = document.createElement("ul");
  o.node.getChildrenUL = function() { return o.node.childrenUL; };
  o.node.li.appendChild(o.node.childrenUL);
  o.node.depth = 0;
  o.node.relpath = relpath;

  getNode(o, o.node);

  o.breadcrumbs = findNavTreePage(toroot, NAVTREE);
  if (o.breadcrumbs == null)
  {
    o.breadcrumbs = findNavTreePage("index.html",NAVTREE);
  }
  if (o.breadcrumbs != null && o.breadcrumbs.length>0)
  {
    var p = o.node;
    for (var i in o.breadcrumbs) 
    {
      var j = o.breadcrumbs[i];
      p = p.children[j];
      expandNode(o,p,true);
    }
    p.itemDiv.className = p.itemDiv.className + " selected";
    p.itemDiv.id = "selected";
    $(window).load(showRoot);
  }
}

