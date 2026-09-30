static const char *console_default_kaixuan_esi_path(const char *argv0,
                                                     char *resolved,
                                                     size_t resolved_size)
{
    char exe_path[PATH_LEN];
    char *slash;
    ssize_t len;

    if (access(DEFAULT_KAIXUAN_ESI, R_OK) == 0) {
        return DEFAULT_KAIXUAN_ESI;
    }

    len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1u);
    if (len > 0) {
        exe_path[len] = '\0';
        slash = strrchr(exe_path, '/');
        if (slash != NULL) {
            *slash = '\0';
            snprintf(resolved, resolved_size, "%s/../%s", exe_path, DEFAULT_KAIXUAN_ESI);
            if (access(resolved, R_OK) == 0) {
                return resolved;
            }
        }
    }

    if (argv0 != NULL && strchr(argv0, '/') != NULL) {
        snprintf(exe_path, sizeof(exe_path), "%s", argv0);
        slash = strrchr(exe_path, '/');
        if (slash != NULL) {
            *slash = '\0';
            snprintf(resolved, resolved_size, "%s/../%s", exe_path, DEFAULT_KAIXUAN_ESI);
            if (access(resolved, R_OK) == 0) {
                return resolved;
            }
        }
    }

    return DEFAULT_KAIXUAN_ESI;
}

static int console_parse_esi_hex(const char *text, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if (text == NULL || value == NULL) {
        return -1;
    }
    errno = 0;
    parsed = strtoul(text, &end, 16);
    if (errno != 0 || end == text || parsed > UINT32_MAX) {
        return -1;
    }
    *value = (uint32_t)parsed;
    return 0;
}

static void console_load_kaixuan_esi(flash_state *state, const char *argv0)
{
    char resolved_path[PATH_LEN];
    const char *path;
    FILE *file;
    char *xml;
    char *vendor;
    char *vendor_end;
    char *vendor_id;
    char *type;
    char *product_code;
    char *revision;
    long file_size;
    size_t bytes_read;
    uint32_t vendor_id_value;
    uint32_t product_code_value;
    uint32_t revision_value;

    path = console_default_kaixuan_esi_path(argv0, resolved_path, sizeof(resolved_path));
    file = fopen(path, "rb");
    if (file == NULL) {
        printf("%s: %s\n", console_text(state,
               "未加载开璇 EtherCAT ESI 文件",
               "Kaixuan EtherCAT ESI file not loaded"), path);
        return;
    }
    if (fseek(file, 0L, SEEK_END) != 0 ||
        (file_size = ftell(file)) <= 0L || file_size > 1024L * 1024L ||
        fseek(file, 0L, SEEK_SET) != 0) {
        fclose(file);
        printf("%s: %s\n", console_text(state,
               "开璇 EtherCAT ESI 文件读取失败",
               "failed to read Kaixuan EtherCAT ESI file"), path);
        return;
    }
    xml = malloc((size_t)file_size + 1u);
    if (xml == NULL) {
        fclose(file);
        printf("%s\n", console_text(state,
               "开璇 EtherCAT ESI 文件内存分配失败",
               "failed to allocate Kaixuan EtherCAT ESI buffer"));
        return;
    }
    bytes_read = fread(xml, 1u, (size_t)file_size, file);
    fclose(file);
    xml[bytes_read] = '\0';
    if (bytes_read != (size_t)file_size) {
        free(xml);
        printf("%s: %s\n", console_text(state,
               "开璇 EtherCAT ESI 文件读取不完整",
               "incomplete Kaixuan EtherCAT ESI file read"), path);
        return;
    }

    vendor = strstr(xml, "<Vendor>");
    vendor_end = vendor == NULL ? NULL : strstr(vendor, "</Vendor>");
    vendor_id = vendor == NULL ? NULL : strstr(vendor, "<Id>#x");
    type = strstr(xml, "<Type ProductCode=\"#x");
    product_code = type == NULL ? NULL : type + strlen("<Type ProductCode=\"#x");
    revision = type == NULL ? NULL : strstr(type, "RevisionNo=\"#x");
    if (revision != NULL) {
        revision += strlen("RevisionNo=\"#x");
    }
    if (vendor_id != NULL) {
        vendor_id += strlen("<Id>#x");
    }

    if (vendor == NULL || vendor_end == NULL || vendor_id == NULL || vendor_id >= vendor_end ||
        product_code == NULL || revision == NULL ||
        console_parse_esi_hex(vendor_id, &vendor_id_value) != 0 ||
        console_parse_esi_hex(product_code, &product_code_value) != 0 ||
        console_parse_esi_hex(revision, &revision_value) != 0) {
        free(xml);
        printf("%s: %s\n", console_text(state,
               "开璇 EtherCAT ESI 格式或身份信息无效",
               "invalid Kaixuan EtherCAT ESI format or identity"), path);
        return;
    }

    printf("%s: %s Vendor=0x%08x Product=0x%08x Revision=0x%08x\n",
           console_text(state, "已加载开璇 EtherCAT ESI", "loaded Kaixuan EtherCAT ESI"),
           path,
           vendor_id_value,
           product_code_value,
           revision_value);
    free(xml);
}
